#!/usr/bin/env python3
"""vs_passthru_check.py - which compression vendor commands work through the Linux nvme driver.

    sudo python3 scripts/vs_passthru_check.py [/dev/nvme1]

Sends the vendor-specific I/O commands with the standard passthrough ioctl
(NVME_IOCTL_IO_CMD, the path nvme-cli io-passthru uses), with the board bound to `nvme`:

  * 84h Commit while RAW, and with a bad handle: the vendor status reaches the caller
  * 81h Compress-to-Staging with a 64-byte metadata buffer for MPTR: does the kernel accept it?
  * 89h FFE-Send 82h with an LBA destination, then 86h Decompress-Read of that extent with
    NOCHK = 1 (no MPTR needed) and without metadata: data must equal the record
  * 86h with NOCHK = 1 and a wrong CSIZE / ULEN: vendor error status
  * the kernel log for warnings about the vendor opcodes

86h with NOCHK = 0 is not sent: without a metadata buffer the kernel leaves MPTR at 0 and the
device would read the expected checksum from host address 0.
Writes only LBAs >= 230000. Restores RAW mode at the end.
"""
import errno, fcntl, os, random, struct, subprocess, sys, zlib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ffe_hwtest import (NVME_IOCTL_IO_CMD, PT, REC, Buf, Dev, T, crb, fletcher4,   # noqa: E402
                        gen, stname, up4k)

LBA = 230000


def io_cmd(d, opc, data=None, dlen=0, meta=None, cdw=(0, 0, 0, 0, 0, 0)):
    """passthrough with an optional metadata buffer; returns (status, dw0, errno or 0)"""
    addr = data.addr if data else 0
    maddr, mlen = (meta.addr, 64) if meta else (0, 0)
    c = bytearray(PT.pack(opc, 0, 0, 1, 0, 0, maddr, addr, mlen, dlen, *cdw, 10000, 0))
    try:
        st = fcntl.ioctl(d.nfd, NVME_IOCTL_IO_CMD, c, True)
    except OSError as e:
        return None, 0, e.errno
    return st & 0x7FF, PT.unpack(c)[-1], 0


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    d = Dev(args[0] if args else "/dev/nvme1")
    t = T()
    rnd = random.Random(3)
    klog0 = len(subprocess.run(["dmesg"], capture_output=True, text=True).stdout.splitlines())

    def show(what, ok, detail):
        t.check(ok, f"{what}: {detail}")
        print(f"  {what:<52} {detail}{'' if ok else '  <-- FAIL'}")

    st, dw0 = d.get_c0()
    if dw0 & 1:
        d.set_c0(0)
    RT = REC if 256 + REC <= d.maxx else d.maxx - 4096
    print(f"vendor commands through NVME_IOCTL_IO_CMD (driver: nvme), host transfer limit "
          f"{d.maxx // 1024} KB\n")
    show(f"host limit allows a {REC // 1024} KB FFE-Send (256 B + record)", True,
         "yes" if 256 + REC <= d.maxx else f"no: {256 + REC} B > {d.maxx} B, FFE records here are {RT // 1024} KB")
    st, _, er = io_cmd(d, 0x84, cdw=(LBA, 0, 0, 0, 0, 0))
    show("84h Commit while RAW", st == 0x00C, f"st={stname(st) if st is not None else 'errno ' + errno.errorcode[er]}")
    if not t.check(d.set_c0(1, 0) == 0, "enable compression"):
        return 1
    st, _, er = io_cmd(d, 0x84, cdw=(LBA, 0, 0, 0x7FFF, 0, 0))
    show("84h Commit, handle 7FFFh", st == 0x1C1, f"st={stname(st) if st is not None else 'errno ' + errno.errorcode[er]}")

    try:
        # ---- 81h with a metadata buffer ----
        rec = gen("text", 4096, rnd)                      # small: rules out the transfer limit
        b = Buf(4096); b.put(rec)
        m = Buf(64); m.put(b"\x5a" * 64)
        st, dw0, er = io_cmd(d, 0x81, data=b, dlen=4096, meta=m, cdw=(4096, 0, 0, 0, 0, 0))
        if er:
            print(f"  {'81h with a 64 B metadata buffer (MPTR)':<52} refused by the kernel: "
                  f"errno {errno.errorcode[er]} ({os.strerror(er)})")
            print(f"  {'':<52} -> 81h cannot be used through the Linux nvme driver")
        else:
            r = crb(m.get(64))
            ok = st == 0 and r["lsize"] == 4096 and r["adler"] == zlib.adler32(rec)
            show("81h with a 64 B metadata buffer (MPTR)", ok,
                 f"accepted: st={stname(st)} handle={dw0 & 0xFFFF} csize={r['csize']}")
            if st == 0:
                st, _, _ = io_cmd(d, 0x84, cdw=(0, 0, 0, (dw0 & 0xFFFF) | 1 << 31, 0, 0))
                show("  84h DISCARD of that record", st == 0, f"st={stname(st)}")

        # ---- FFE-Send to LBA, then 86h with NOCHK ----
        for kind in ("text", "runs", "zeros"):
            rec = gen(kind, RT, rnd)
            lid = 0x300 + rnd.randrange(0x100)
            st, _ = d.ffe_send(lid, 0x82, rec)
            st2, got, out = d.ffe_recv(lid, 64 + RT + 4096)
            r = crb(out)
            nlb = r["psize"] // 4096
            if not t.check(st == 0 and st2 == 0 and not r["flags"] & 1, f"{kind}: FFE 82h st={stname(st)}/{stname(st2)}"):
                continue
            st, _ = d.ffe_send(lid + 1, 0x82, rec, dest=(LBA, nlb))
            st2, _, _ = d.ffe_recv(lid + 1, 4096)
            disk = d.read_lba(LBA, nlb)
            t.check(st == 0 and st2 == 0 and fletcher4(disk) == r["fl"], f"{kind}: FFE 82h to LBA {LBA}")
            o = Buf(RT); o.put(b"\xa5" * RT)
            st, _, er = io_cmd(d, 0x86, data=o, dlen=RT, cdw=(LBA, 0, nlb - 1, r["csize"], RT, 1))
            show(f"86h NOCHK, {kind} {RT // 1024} KB ({r['csize']} B, {nlb} blocks)", st == 0 and o.get(RT) == rec,
                 f"st={stname(st) if st is not None else 'errno ' + errno.errorcode[er]} "
                 f"data {'== record' if o.get(RT) == rec else '!= record'}")
            if kind == "text":
                st, _, _ = io_cmd(d, 0x86, data=o, dlen=RT, cdw=(LBA, 0, nlb - 1, r["csize"], RT - 4096, 1))
                show("86h NOCHK, ULEN 4 KB short", st == 0x1C4, f"st={stname(st)}")
                st, _, _ = io_cmd(d, 0x86, data=o, dlen=RT, cdw=(LBA, 0, nlb - 1, 0, RT, 1))
                show("86h NOCHK, CSIZE 0", st == 0x002, f"st={stname(st)}")
                bad = bytearray(disk); bad[200:208] = b"\xff" * 8
                wb = Buf(len(bad)); wb.put(bytes(bad))
                fd = os.open(d.nsdev, os.O_WRONLY | os.O_DIRECT)
                os.pwritev(fd, [wb.m], LBA * 4096)
                os.close(fd)
                st, _, _ = io_cmd(d, 0x86, data=o, dlen=RT, cdw=(LBA, 0, nlb - 1, r["csize"], RT, 1))
                show("86h NOCHK, corrupted stream", st in (0x1C3, 0x1C4), f"st={stname(st)}")
        # ---- full 128 KB record: software zlib stream written with a plain Write ----
        for lvl in (1, 6, 9):
            rec = gen("text", REC, rnd)
            z = zlib.compress(rec, lvl)
            nlb = up4k(len(z)) // 4096
            wb = Buf(nlb * 4096); wb.put(z)
            fd = os.open(d.nsdev, os.O_WRONLY | os.O_DIRECT)
            os.pwritev(fd, [wb.m], (LBA + 64) * 4096)
            os.close(fd)
            o = Buf(REC); o.put(b"\xa5" * REC)
            st, _, er = io_cmd(d, 0x86, data=o, dlen=REC, cdw=(LBA + 64, 0, nlb - 1, len(z), REC, 1))
            show(f"86h NOCHK, 128 KB record, zlib level {lvl} ({len(z)} B)", st == 0 and o.get(REC) == rec,
                 f"st={stname(st) if st is not None else 'errno ' + errno.errorcode[er]} "
                 f"data {'== record' if o.get(REC) == rec else '!= record'}")
    finally:
        st = d.set_c0(0)
        print("\ncompression disabled (RAW)" if st == 0 else f"\ndisable failed: {stname(st)}")

    new = subprocess.run(["dmesg"], capture_output=True, text=True).stdout.splitlines()[klog0:]
    bad = [l for l in new if "nvme" in l.lower() or "AER" in l or "DMAR" in l]
    print("new kernel messages:", *(bad or ["(none)"]), sep="\n  ")
    t.check(not any("DMAR" in l or "AER" in l or "timeout" in l.lower() for l in bad), "kernel errors")
    print(f"\n{t.ok} checks passed, {t.bad} failed -> {'PASS' if t.bad == 0 else 'FAIL'}")
    return 0 if t.bad == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
