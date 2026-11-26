#!/usr/bin/env python3
"""ffe_hwtest.py - test the compression engine on the ZC706 board through Linux NVMe passthrough.

    sudo python3 scripts/ffe_hwtest.py [/dev/nvme1] [--keep-enabled]

Uses FFE-Send (89h) / FFE-Receive (8Ah) from docs/compression_spec.md. Those return the
CRB in the data buffer. Compress-to-Staging (81h) and Decompress-Read (86h) are not used:
they need MPTR, and Linux rejects metadata passthrough on a namespace without integrity.

Checks, against Python's zlib:
  * Get Features C0h (RAW after reset), FFE-Send refused while RAW, Set Features C0h CEN=1
  * 82h compress (inline, retained): CRB sizes, Adler-32, Fletcher-4, zlib.decompress of
    the hardware stream == record, ZFS 12.5% rule, incompressible records stored raw
  * 83h decompress (inline) of the hardware stream and of zlib streams made in software
    (levels 0, 1, 6, 9: stored, fixed and dynamic Huffman blocks)
  * 82h with an LBA destination, read back with a plain O_DIRECT read, then 83h with an
    LBA source
  * corrupt stream -> Decompression Error (C3h), unknown List ID -> C1h
  * 128 KB compress / decompress round-trip rate (FFE-Send + FFE-Receive, PCIe included)
Restores RAW mode at the end unless --keep-enabled. Writes only LBAs >= 200000.
"""
import ctypes, fcntl, mmap, os, random, struct, sys, time, zlib

NVME_IOCTL_ADMIN_CMD = 0xC0484E41   # _IOWR('N', 0x41, struct nvme_passthru_cmd)
NVME_IOCTL_IO_CMD    = 0xC0484E43   # _IOWR('N', 0x43, struct nvme_passthru_cmd)
PT = struct.Struct("<BBHIIIQQII6III")   # struct nvme_passthru_cmd, 72 bytes
assert PT.size == 72

OPC_FSEND, OPC_FRECV = 0x89, 0x8A
REC = 128 * 1024
TEST_LBA = 200000
SC = {0x00C: "Command Sequence Error", 0x002: "Invalid Field", 0x1C0: "Staging Full",
      0x1C1: "Invalid Handle", 0x1C2: "Checksum Mismatch", 0x1C3: "Decompression Error",
      0x1C4: "Length Mismatch"}


class Buf:
    """page-aligned host buffer"""
    def __init__(self, n):
        self.n = max(4096, (n + 4095) & ~4095)
        self.m = mmap.mmap(-1, self.n)
        self.addr = ctypes.addressof(ctypes.c_char.from_buffer(self.m))

    def put(self, b):
        self.m[:len(b)] = b
        if len(b) < self.n:
            self.m[len(b):] = bytes(self.n - len(b))

    def get(self, n):
        return bytes(self.m[:n])


class Dev:
    def __init__(self, ctrl):
        self.cfd = os.open(ctrl, os.O_RDWR)
        self.nsdev = ctrl + "n1"
        self.nfd = os.open(self.nsdev, os.O_RDWR)

    def cmd(self, io, opc, nsid=0, cdw2=0, addr=0, dlen=0, cdw=(0, 0, 0, 0, 0, 0), timeout_ms=10000):
        c = bytearray(PT.pack(opc, 0, 0, nsid, cdw2, 0, 0, addr, 0, dlen, *cdw, timeout_ms, 0))
        st = fcntl.ioctl(self.nfd if io else self.cfd, NVME_IOCTL_IO_CMD if io else NVME_IOCTL_ADMIN_CMD, c, True)
        return st & 0x7FF, PT.unpack(c)[-1]

    def get_c0(self):
        return self.cmd(False, 0x0A, cdw=(0xC0, 0, 0, 0, 0, 0))

    def set_c0(self, en, rs=0):
        return self.cmd(False, 0x09, cdw=(0xC0, (rs << 4) | en, 0, 0, 0, 0))[0]

    def ffe_send(self, lid, fn, pay=b"", dest=None, src_lba=None, thr=0, ioff=256):
        """fn 0x82 compress / 0x83 decompress; dest = (slba, nlb) or None; src_lba = (slba, nlb) or None"""
        lba_src = src_lba is not None
        plen = 256 if lba_src else (ioff + len(pay) + 127) & ~127
        pb = bytearray(plen)
        pb[0] = 0x01                                          # destination
        if dest:
            pb[1] = 1
            struct.pack_into("<QH", pb, 12, dest[0], dest[1] - 1)
        else:
            struct.pack_into("<H", pb, 20, 0)
        if lba_src:                                           # LBA source
            pb[64] = 0x03
            struct.pack_into("<IQH", pb, 68, 1, src_lba[0], src_lba[1] - 1)
        else:                                                 # inline source
            pb[64] = 0x04
            struct.pack_into("<II", pb, 68, len(pay), ioff)
            pb[ioff:ioff + len(pay)] = pay
        pb[128] = fn                                          # function descriptor
        pb[134] = 1                                           # 1 source
        pb[144] = 1                                           # source descriptor index 1
        pb[192] = 0xFE                                        # function parameter descriptor
        if fn == 0x82:
            pb[197], pb[198], pb[199] = 0x01, 6, thr
        else:
            pb[196] = 0x01
        b = Buf(plen)
        b.put(pb)
        return self.cmd(True, OPC_FSEND, nsid=1, cdw2=lid, addr=b.addr, dlen=plen,
                        cdw=(plen, (2 << 16) | 2, 0 if lba_src else len(pay), 0, 0, 0))

    def ffe_recv(self, lid, n):
        b = Buf(n)
        b.put(b"\x5a" * n)
        st, got = self.cmd(True, OPC_FRECV, nsid=1, cdw2=lid, addr=b.addr, dlen=n, cdw=(n, 0, 0, 0, 0, 0))
        return st, got, b.get(n)

    def read_lba(self, slba, nlb):
        b = Buf(nlb * 4096)
        fd = os.open(self.nsdev, os.O_RDONLY | os.O_DIRECT)
        try:
            n = os.preadv(fd, [b.m], slba * 4096)
        finally:
            os.close(fd)
        return b.get(n)


def crb(b):
    w = struct.unpack_from("<IIII4QIHH", b, 0)
    return dict(handle=w[0] & 0xFFFF, flags=w[0] >> 16, lsize=w[1], csize=w[2], psize=w[3],
                fl=tuple(w[4:8]), adler=w[8], lid=w[9], status=w[10])


def fletcher4(b):
    m = (1 << 64) - 1
    a = bb = c = d = 0
    for (x,) in struct.iter_unpack("<I", b):
        a = (a + x) & m; bb = (bb + a) & m; c = (c + bb) & m; d = (d + c) & m
    return (a, bb, c, d)


def up4k(n):
    return (n + 4095) & ~4095


def gen(kind, n, rnd):
    if kind == "zeros":
        return bytes(n)
    if kind == "random":
        return rnd.randbytes(n)
    if kind == "text":
        w = [b"zfs", b"record", b"block", b"nvme", b"fpga", b"the", b"of", b"data", b"\n", b"compress"]
        out = bytearray()
        while len(out) < n:
            out += rnd.choice(w) + b" "
        return bytes(out[:n])
    out = bytearray()                                         # runs
    while len(out) < n:
        out += bytes([rnd.randrange(256)]) * rnd.randrange(1, 300)
    return bytes(out[:n])


class T:
    def __init__(self):
        self.ok = self.bad = 0

    def check(self, cond, msg):
        if cond:
            self.ok += 1
        else:
            self.bad += 1
            print("  FAIL:", msg)
        return cond


def stname(st):
    return f"{st:03x}" + (f" ({SC[st]})" if st in SC else "")


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    keep = "--keep-enabled" in sys.argv
    d = Dev(args[0] if args else "/dev/nvme1")
    t = T()
    rnd = random.Random(1)
    lid = 0x100

    # ---- mode ----
    st, dw0 = d.get_c0()
    nslot = dw0 >> 16
    print(f"Get Features C0h: st={stname(st)} CEN={dw0 & 1} MAXRS={(dw0 >> 12) & 15} NSLOT={nslot}")
    t.check(st == 0 and nslot > 0, "Get Features C0h")
    if dw0 & 1 == 0:
        st, _ = d.ffe_send(0x77, 0x82, b"x" * 4096)
        t.check(st == 0x00C, f"FFE-Send while RAW should be Command Sequence Error, got {stname(st)}")
    t.check(d.set_c0(1, 1) == 0x002, "RECSZ above the build limit should be Invalid Field")
    st = d.set_c0(1, 0)
    if not t.check(st == 0, f"enable compression: {stname(st)}"):
        return 1
    print("compression enabled (CEN=1, 128 KB records)\n")

    try:
        # ---- 82h retained + 83h inline, against Python zlib ----
        print(f"{'record':>8} {'bytes':>7} {'csize':>7} {'psize':>7} {'ratio':>6}  sw zlib-6  result")
        keep_ext = None
        for kind in ("zeros", "text", "runs", "random"):
            for n in (512, 4096, 13824, 65536, REC):
                rec = gen(kind, n, rnd)
                lid += 1
                bad0 = t.bad
                st, _ = d.ffe_send(lid, 0x82, rec)
                if not t.check(st == 0, f"{kind}/{n}: FFE-Send 82h {stname(st)}"):
                    continue
                st, got, out = d.ffe_recv(lid, 64 + up4k(n) + 4096)
                r = crb(out)
                inc = r["flags"] & 1
                t.check(st == 0, f"{kind}/{n}: FFE-Receive {stname(st)}")
                t.check(got == 64 + r["psize"], f"{kind}/{n}: received {got} bytes, psize {r['psize']}")
                t.check(r["lsize"] == n and r["lid"] == lid, f"{kind}/{n}: CRB lsize {r['lsize']} lid {r['lid']:x}")
                t.check(r["psize"] == up4k(r["csize"]), f"{kind}/{n}: psize {r['psize']} for csize {r['csize']}")
                t.check(r["adler"] == zlib.adler32(rec), f"{kind}/{n}: Adler-32 {r['adler']:08x} != {zlib.adler32(rec):08x}")
                stored = out[64:64 + r["psize"]]
                t.check(fletcher4(stored) == r["fl"], f"{kind}/{n}: Fletcher-4 of the stored bytes")
                if inc:
                    t.check(r["csize"] == n and stored[:n] == rec, f"{kind}/{n}: incompressible record not stored raw")
                    t.check(kind == "random", f"{kind}/{n}: unexpectedly incompressible")
                else:
                    t.check(r["csize"] <= n - n // 8, f"{kind}/{n}: kept compressed above the 12.5% rule")
                    try:
                        sw = zlib.decompress(stored[:r["csize"]])
                        t.check(sw == rec, f"{kind}/{n}: zlib.decompress(hw stream) != record")
                    except zlib.error as e:
                        t.check(False, f"{kind}/{n}: Python zlib rejects the hw stream: {e}")
                    t.check(not any(stored[r["csize"]:]), f"{kind}/{n}: padding not zero")
                    # hardware decompress of its own stream (inline)
                    lid += 1
                    st, _ = d.ffe_send(lid, 0x83, stored[:r["csize"]])
                    if t.check(st == 0, f"{kind}/{n}: FFE-Send 83h {stname(st)}"):
                        st, got, o2 = d.ffe_recv(lid, 64 + n)
                        t.check(st == 0 and got == 64 + n and o2[64:64 + n] == rec,
                                f"{kind}/{n}: 83h round trip st={stname(st)} got={got}")
                    if kind == "runs" and n == REC:
                        keep_ext = (rec, stored, r)
                sw6 = len(zlib.compress(rec, 6))
                print(f"{kind:>8} {n:>7} {r['csize']:>7} {r['psize']:>7} {n / r['csize']:>5.2f}x {sw6:>9}  "
                      f"{'INCOMP' if inc else 'ok'}{'' if t.bad == bad0 else '  <-- FAIL'}")

        # ---- software zlib streams through 83h ----
        print("\nsoftware zlib streams -> 83h:")
        for kind in ("text", "runs", "random"):
            rec = gen(kind, REC, rnd)
            for lvl in (0, 1, 6, 9):
                u = rec if lvl else rec[:65536]   # inline stream must fit a record: stored blocks at 64 KB
                z = zlib.compress(u, lvl)
                if len(z) > REC:
                    continue
                lid += 1
                st, _ = d.ffe_send(lid, 0x83, z)
                ok = st == 0
                if ok:
                    st, got, o = d.ffe_recv(lid, 64 + len(u))
                    ok = st == 0 and o[64:64 + len(u)] == u
                t.check(ok, f"zlib level {lvl} {kind}: st={stname(st)}")
                print(f"  {kind:>6} level {lvl}: {len(u):>7} -> {len(z):>7} B  {'ok' if ok else 'FAIL'}")

        # ---- LBA destination / LBA source ----
        print("\nLBA destination and source:")
        if keep_ext:
            rec, stored, r = keep_ext
            nlb = r["psize"] // 4096
            lid += 1
            st, _ = d.ffe_send(lid, 0x82, rec, dest=(TEST_LBA, nlb))
            t.check(st == 0, f"82h -> LBA {TEST_LBA}: {stname(st)}")
            st, got, _ = d.ffe_recv(lid, 4096)
            t.check(st == 0 and got == 64, f"Receive after LBA destination: st={stname(st)} got={got}")
            disk = d.read_lba(TEST_LBA, nlb)
            t.check(disk == stored, "O_DIRECT read of the destination LBAs != retained stored bytes")
            lid += 1
            st, _ = d.ffe_send(lid, 0x83, src_lba=(TEST_LBA, nlb))
            ok = st == 0
            if ok:
                st, got, o = d.ffe_recv(lid, 64 + REC)
                ok = st == 0 and o[64:64 + REC] == rec
            t.check(ok, f"83h from LBA source: st={stname(st)}")
            print(f"  {nlb} blocks at LBA {TEST_LBA}: write {'ok' if disk == stored else 'FAIL'}, "
                  f"decompress from LBA {'ok' if ok else 'FAIL'}")

        # ---- errors ----
        print("\nerror handling:")
        z = bytearray(zlib.compress(gen("text", 65536, rnd), 6))
        z[40:48] = b"\xff" * 8
        lid += 1
        st, _ = d.ffe_send(lid, 0x83, bytes(z))
        if st == 0:
            st, _, _ = d.ffe_recv(lid, 64 + 65536)
        t.check(st in (0x1C3, 0x1C4), f"corrupt stream: {stname(st)}")
        print(f"  corrupt stream -> {stname(st)}")
        st, _, _ = d.ffe_recv(0x0999, 4096)
        t.check(st == 0x1C1, f"unknown List ID: {stname(st)}")
        print(f"  unknown List ID -> {stname(st)}")

        # ---- rate ----
        rec = gen("text", REC, rnd)
        N = 200
        t0 = time.perf_counter()
        for _ in range(N):
            lid = 0x200 + (lid + 1) % 0x100
            d.ffe_send(lid, 0x82, rec)
            st, _, out = d.ffe_recv(lid, 64 + REC + 4096)
        t1 = time.perf_counter()
        cs = crb(out)["csize"]
        z = out[64:64 + cs]
        t2 = time.perf_counter()
        for _ in range(N):
            lid = 0x200 + (lid + 1) % 0x100
            d.ffe_send(lid, 0x83, z)
            st, _, o = d.ffe_recv(lid, 64 + REC)
        t3 = time.perf_counter()
        t.check(o[64:64 + REC] == rec, "rate loop: last decompress")
        mb = N * REC / 1e6
        print(f"\nrate, 128 KB text records, one at a time (FFE-Send + FFE-Receive, Python ioctl):")
        print(f"  compress   {mb / (t1 - t0):7.1f} MB/s  ({(t1 - t0) / N * 1e6:.0f} us/record)")
        print(f"  decompress {mb / (t3 - t2):7.1f} MB/s  ({(t3 - t2) / N * 1e6:.0f} us/record)")
        sw0 = time.perf_counter()
        for _ in range(20):
            zlib.compress(rec, 6)
        sw1 = time.perf_counter()
        print(f"  (Python zlib level 6 on this CPU: {20 * REC / 1e6 / (sw1 - sw0):.1f} MB/s)")
    finally:
        if not keep:
            st = d.set_c0(0)
            print("\ncompression disabled (RAW)" if st == 0 else f"\ndisable failed: {stname(st)}")

    print(f"\n{t.ok} checks passed, {t.bad} failed -> {'PASS' if t.bad == 0 else 'FAIL'}")
    return 0 if t.bad == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
