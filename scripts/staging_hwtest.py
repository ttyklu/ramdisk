#!/usr/bin/env python3
"""staging_hwtest.py - test the two-phase compressed write on the ZC706 board.

    sudo python3 scripts/staging_hwtest.py [0000:08:00.0] [--keep-vfio]

Compress-to-Staging (81h) writes a 64-byte Result Record to MPTR, and Decompress-Read (86h)
reads the expected Fletcher-4 from MPTR. Linux refuses passthrough metadata on a namespace
without integrity, so this test drives the controller from user space through VFIO:

  * unbinds the board from nvme, binds it to vfio-pci (it must be alone in its IOMMU group)
  * brings the controller up itself (admin queue, one polled I/O queue pair, no interrupts)
  * runs the tests below, then hands the board back to the nvme driver (unless --keep-vfio)

The namespace must not be mounted. Writes only LBAs >= 200000.

Checks, against Python's zlib:
  * 81h / 84h refused with Command Sequence Error while RAW; Set Features C0h CEN=1
  * 81h for zeros / text / runs / random records, 512 B ... 128 KB: CQE handle + INCOMP,
    Result Record (VALID, LSIZE, CSIZE, PSIZE, Adler-32, ZFS 12.5% rule)
  * 84h Commit, then a plain Read of the LBAs: Fletcher-4 matches the record, zlib.decompress
    of the stored stream == record, zero padding, raw data for incompressible records, the
    block after the extent untouched
  * 86h Decompress-Read of each committed extent == record
  * threshold percent T, several records staged and committed out of order
  * errors: bad LSIZE, wrong NLB (slot kept), bad / stale handle, LBA range, DISCARD,
    Staging Full after NSLOT records, Checksum Mismatch, NOCHK, wrong ULEN
  * 128 KB stage + commit and decompress-read rates
"""
import ctypes, fcntl, mmap, os, random, struct, subprocess, sys, time, zlib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ffe_hwtest import SC, T, crb, fletcher4, gen, stname, up4k    # noqa: E402

REC = 128 * 1024
TEST_LBA = 200000
OPC_C2S, OPC_COMMIT, OPC_DREAD = 0x81, 0x84, 0x86
ST_INVALID_FIELD, ST_LBA_RANGE, ST_CMD_SEQ = 0x002, 0x080, 0x00C
ST_FULL, ST_HANDLE, ST_CKSUM, ST_DECOMP, ST_LEN = 0x1C0, 0x1C1, 0x1C2, 0x1C3, 0x1C4

# ---- VFIO ioctls (linux/vfio.h: _IO(';', 100 + n)) ----
VFIO_GET_API_VERSION, VFIO_CHECK_EXTENSION, VFIO_SET_IOMMU = 0x3B64, 0x3B65, 0x3B66
VFIO_GROUP_GET_STATUS, VFIO_GROUP_SET_CONTAINER, VFIO_GROUP_GET_DEVICE_FD = 0x3B67, 0x3B68, 0x3B6A
VFIO_DEVICE_GET_REGION_INFO, VFIO_IOMMU_MAP_DMA, VFIO_IOMMU_UNMAP_DMA = 0x3B6C, 0x3B71, 0x3B72
VFIO_TYPE1_IOMMU, VFIO_TYPE1v2_IOMMU = 1, 3
BAR0, CONFIG = 0, 7

# ---- DMA memory map (IOVA = IOVA_BASE + offset) ----
IOVA_BASE, MEM_SIZE = 0x1000_0000, 4 << 20
ASQ, ACQ, IOSQ, IOCQ = 0x0000, 0x1000, 0x2000, 0x3000
IDENT, GUARD, RES2 = 0x4000, 0x5000, 0x6000
RES = 0x8000                               # Result Records, 64 B each, up to 512
PRPL_A, PRPL_B = 0x10000, 0x11000          # PRP list pages
BUF_A, BUF_B = 0x100000, 0x200000          # 1 MiB each
QD = 64


def wr(path, s):
    with open(path, "w") as f:
        f.write(s)


class Vfio:
    def __init__(self, bdf):
        self.bdf = bdf
        grp = os.path.basename(os.readlink(f"/sys/bus/pci/devices/{bdf}/iommu_group"))
        self.cfd = os.open("/dev/vfio/vfio", os.O_RDWR)
        if fcntl.ioctl(self.cfd, VFIO_GET_API_VERSION) != 0:
            raise RuntimeError("unexpected VFIO API version")
        self.gfd = os.open(f"/dev/vfio/{grp}", os.O_RDWR)
        stb = bytearray(struct.pack("<II", 8, 0))
        fcntl.ioctl(self.gfd, VFIO_GROUP_GET_STATUS, stb, True)
        if not struct.unpack("<II", stb)[1] & 1:
            raise RuntimeError(f"IOMMU group {grp} not viable: other devices in it are not bound to vfio")
        fcntl.ioctl(self.gfd, VFIO_GROUP_SET_CONTAINER, bytearray(struct.pack("<i", self.cfd)), True)
        typ = VFIO_TYPE1v2_IOMMU if fcntl.ioctl(self.cfd, VFIO_CHECK_EXTENSION, VFIO_TYPE1v2_IOMMU) else VFIO_TYPE1_IOMMU
        fcntl.ioctl(self.cfd, VFIO_SET_IOMMU, typ)
        self.dfd = fcntl.ioctl(self.gfd, VFIO_GROUP_GET_DEVICE_FD, bytearray(bdf.encode() + b"\0"), True)
        self.bar0 = self.region(BAR0)
        self.cfg = self.region(CONFIG)
        # DMA memory
        self.mem = mmap.mmap(-1, MEM_SIZE)
        self.mem[:] = bytes(MEM_SIZE)
        self._ref = ctypes.c_char.from_buffer(self.mem)
        m = bytearray(struct.pack("<IIQQQ", 32, 3, ctypes.addressof(self._ref), IOVA_BASE, MEM_SIZE))
        fcntl.ioctl(self.cfd, VFIO_IOMMU_MAP_DMA, m, True)
        # memory space + bus master
        cmdreg = struct.unpack("<H", os.pread(self.dfd, 2, self.cfg + 4))[0]
        os.pwrite(self.dfd, struct.pack("<H", cmdreg | 0x6), self.cfg + 4)

    def region(self, idx):
        b = bytearray(struct.pack("<IIIIQQ", 32, 0, idx, 0, 0, 0))
        fcntl.ioctl(self.dfd, VFIO_DEVICE_GET_REGION_INFO, b, True)
        return struct.unpack("<IIIIQQ", b)[5]

    def r32(self, off):
        return struct.unpack("<I", os.pread(self.dfd, 4, self.bar0 + off))[0]

    def w32(self, off, v):
        os.pwrite(self.dfd, struct.pack("<I", v & 0xFFFFFFFF), self.bar0 + off)

    def close(self):
        try:
            cmdreg = struct.unpack("<H", os.pread(self.dfd, 2, self.cfg + 4))[0]
            os.pwrite(self.dfd, struct.pack("<H", cmdreg & ~0x4), self.cfg + 4)   # bus master off
        except OSError:
            pass
        u = bytearray(struct.pack("<IIQQ", 24, 0, IOVA_BASE, MEM_SIZE))
        try:
            fcntl.ioctl(self.cfd, VFIO_IOMMU_UNMAP_DMA, u, True)
        except OSError:
            pass
        for fd in (self.dfd, self.gfd, self.cfd):
            os.close(fd)


class Nvme:
    """minimal polled NVMe controller driver: admin queue + I/O queue pair 1"""

    def __init__(self, v):
        self.v = v
        cap = v.r32(0) | v.r32(4) << 32
        self.db = 4 << ((cap >> 32) & 0xF)
        self.tmo = max(1, (cap >> 24) & 0xFF) * 0.5
        self.q = {0: [ASQ, ACQ, 0, 0, 1], 1: [IOSQ, IOCQ, 0, 0, 1]}   # sq, cq, tail, head, phase
        self.cid = 0
        cc = v.r32(0x14)
        if cc & 1:
            v.w32(0x14, cc & ~1)
        self._wait(lambda: v.r32(0x1C) & 1 == 0, "CSTS.RDY=0")
        v.w32(0x24, (QD - 1) << 16 | (QD - 1))
        for reg, off in ((0x28, ASQ), (0x30, ACQ)):
            v.w32(reg, IOVA_BASE + off)
            v.w32(reg + 4, 0)
        v.w32(0x14, 4 << 20 | 6 << 16 | 1)
        self._wait(lambda: v.r32(0x1C) & 3 != 0, "CSTS.RDY=1")
        if v.r32(0x1C) & 2:
            raise RuntimeError("controller fatal status (CSTS.CFS)")
        st, _ = self.admin(0x06, prp=self.prp(IDENT, 4096), cdw=(1,))
        idb = self.get(IDENT, 4096)
        self.model = idb[24:64].decode(errors="replace").strip()
        self.fw = idb[64:72].decode(errors="replace").strip()
        self.mdts = idb[77]
        self.nsze = None
        if self.admin(0x06, nsid=1, prp=self.prp(IDENT, 4096), cdw=(0,))[0] == 0:
            self.nsze = struct.unpack_from("<Q", self.get(IDENT, 8))[0]
        for opc, c10, c11 in ((0x05, (QD - 1) << 16 | 1, 1), (0x01, (QD - 1) << 16 | 1, 1 << 16 | 1)):
            st, _ = self.admin(opc, prp=(IOVA_BASE + (IOCQ if opc == 5 else IOSQ), 0), cdw=(c10, c11))
            if st:
                raise RuntimeError(f"create I/O queue opc {opc:02x}: {stname(st)}")

    def _wait(self, cond, what):
        t = time.time() + self.tmo
        while not cond():
            if time.time() > t:
                raise RuntimeError(f"timeout waiting for {what}")
            time.sleep(0.001)

    # host memory
    def put(self, off, b):
        self.v.mem[off:off + len(b)] = b

    def get(self, off, n):
        return bytes(self.v.mem[off:off + n])

    def prp(self, off, n, lst=PRPL_A):
        a = IOVA_BASE + off
        pages = max(1, (n + 4095) // 4096)
        if pages == 1:
            return a, 0
        if pages == 2:
            return a, a + 4096
        self.put(lst, struct.pack(f"<{pages - 1}Q", *(a + 4096 * k for k in range(1, pages))))
        return a, IOVA_BASE + lst

    def submit(self, qid, opc, nsid=0, mptr=0, prp=(0, 0), cdw=(), timeout=10.0):
        q = self.q[qid]
        self.cid = (self.cid + 1) & 0xFFFF
        c = list(cdw) + [0] * (6 - len(cdw))
        self.put(q[0] + q[2] * 64, struct.pack("<BBHIIIQQQ6I", opc, 0, self.cid, nsid, 0, 0, mptr, prp[0], prp[1], *c))
        q[2] = (q[2] + 1) % QD
        self.v.w32(0x1000 + 2 * qid * self.db, q[2])
        off = q[1] + q[3] * 16
        t = time.time() + timeout
        while True:
            dw0, _, _, _, cid, sf = struct.unpack("<IIHHHH", self.v.mem[off:off + 16])
            if sf & 1 == q[4]:
                break
            if time.time() > t:
                raise RuntimeError(f"no completion for opc {opc:02x} on queue {qid} "
                                   f"(CSTS {self.v.r32(0x1C):08x})")
        q[3] += 1
        if q[3] == QD:
            q[3], q[4] = 0, q[4] ^ 1
        self.v.w32(0x1000 + (2 * qid + 1) * self.db, q[3])
        if cid != self.cid:
            raise RuntimeError(f"completion cid {cid} != submitted {self.cid}")
        return (sf >> 1) & 0x7FF, dw0

    def admin(self, opc, **kw):
        return self.submit(0, opc, **kw)

    def io(self, opc, **kw):
        return self.submit(1, opc, nsid=1, **kw)

    # commands
    def get_c0(self):
        return self.admin(0x0A, cdw=(0xC0,))

    def set_c0(self, en, rs=0):
        return self.admin(0x09, cdw=(0xC0, rs << 4 | en))[0]

    def c2s(self, rec, thr=0, res=RES, lsize=None):
        self.put(BUF_A, rec)
        self.put(res, b"\x5a" * 64)
        n = len(rec) if lsize is None else lsize
        st, dw0 = self.io(OPC_C2S, mptr=IOVA_BASE + res, prp=self.prp(BUF_A, max(n, 1)), cdw=(n, thr))
        return st, dw0 & 0xFFFF, (dw0 >> 16) & 1, crb(self.get(res, 64))

    def commit(self, h, slba, nlb, discard=False):
        return self.io(OPC_COMMIT, cdw=(slba & 0xFFFFFFFF, slba >> 32, nlb - 1, h | (discard << 31)))[0]

    def rw(self, write, slba, nlb, off=BUF_B, lst=PRPL_B):
        return self.io(0x01 if write else 0x02, prp=self.prp(off, nlb * 4096, lst),
                       cdw=(slba & 0xFFFFFFFF, slba >> 32, nlb - 1))[0]

    def dread(self, slba, nlb, csize, ulen, res=RES, nochk=False):
        self.put(BUF_B, b"\x5a" * ulen)
        st, _ = self.io(OPC_DREAD, mptr=IOVA_BASE + res, prp=self.prp(BUF_B, ulen, PRPL_B),
                        cdw=(slba & 0xFFFFFFFF, slba >> 32, nlb - 1, csize, ulen, int(nochk)))
        return st, self.get(BUF_B, ulen)

    def shutdown(self):
        for opc in (0x00, 0x04):                      # delete I/O SQ, CQ
            self.admin(opc, cdw=(1,))
        self.v.w32(0x14, self.v.r32(0x14) & ~1)   # no SHN support in the RTL: just disable
        self._wait(lambda: self.v.r32(0x1C) & 1 == 0, "CSTS.RDY=0")


def find_board():
    for d in sorted(os.listdir("/sys/bus/pci/devices")):
        p = f"/sys/bus/pci/devices/{d}"
        if open(p + "/vendor").read().strip() == "0x10ee" and open(p + "/class").read().startswith("0x0108"):
            return d
    raise SystemExit("no Xilinx NVMe board found")


def drv(bdf):
    p = f"/sys/bus/pci/devices/{bdf}/driver"
    return os.path.basename(os.readlink(p)) if os.path.exists(p) else None


def to_vfio(bdf):
    p = f"/sys/bus/pci/devices/{bdf}"
    grp = os.readlink(p + "/iommu_group")
    others = [d for d in os.listdir(p + "/iommu_group/devices") if d != bdf]
    if others:
        raise SystemExit(f"IOMMU group {os.path.basename(grp)} also holds {others}; refusing")
    if drv(bdf) == "nvme":
        ctrls = os.listdir(p + "/nvme")
        mounts = open("/proc/mounts").read()
        for c in ctrls:
            for ns in os.listdir(f"{p}/nvme/{c}"):
                if ns.startswith(c + "n") and f"/dev/{ns}" in mounts:
                    raise SystemExit(f"/dev/{ns} is mounted; unmount it first")
    subprocess.run(["modprobe", "vfio-pci"], check=True)
    wr(p + "/driver_override", "vfio-pci")
    if drv(bdf):
        wr(p + "/driver/unbind", bdf)
    wr("/sys/bus/pci/drivers_probe", bdf)
    if drv(bdf) != "vfio-pci":
        raise SystemExit(f"bind to vfio-pci failed (driver now {drv(bdf)})")


def to_nvme(bdf):
    p = f"/sys/bus/pci/devices/{bdf}"
    if drv(bdf):
        wr(p + "/driver/unbind", bdf)
    wr(p + "/driver_override", "\n")
    wr("/sys/bus/pci/drivers_probe", bdf)


def tests(d, t, rnd):
    st, dw0 = d.get_c0()
    nslot = dw0 >> 16
    print(f"Get Features C0h: st={stname(st)} CEN={dw0 & 1} MAXRS={(dw0 >> 12) & 15} NSLOT={nslot}")
    t.check(st == 0 and nslot > 0, "Get Features C0h")
    if dw0 & 1:
        t.check(d.set_c0(0) == 0, "set RAW")
    st, *_ = d.c2s(b"x" * 4096)
    t.check(st == ST_CMD_SEQ, f"81h while RAW should be Command Sequence Error, got {stname(st)}")
    t.check(d.commit(0, TEST_LBA, 1) == ST_CMD_SEQ, "84h while RAW should be Command Sequence Error")
    st = d.set_c0(1, 0)
    if not t.check(st == 0, f"enable compression: {stname(st)}"):
        return
    print("compression enabled (CEN=1, 128 KB records)\n")

    lba = [TEST_LBA]

    def alloc(nlb):
        s = lba[0]
        lba[0] += nlb + 1                                 # one guard block after each extent
        return s

    # ---- stage, commit, read back, decompress-read ----
    print(f"{'record':>8} {'bytes':>7} {'csize':>7} {'psize':>7} {'ratio':>6} {'sw zlib-6':>9}  result")
    ext = []
    for kind in ("zeros", "text", "runs", "random"):
        for n in (512, 4096, 13824, 65536, REC):
            rec = gen(kind, n, rnd)
            tag = f"{kind}/{n}"
            bad0 = t.bad
            st, h, inc, r = d.c2s(rec)
            if not t.check(st == 0, f"{tag}: 81h {stname(st)}"):
                continue
            t.check(r["flags"] & 2 and not r["flags"] & 4, f"{tag}: Result Record flags {r['flags']:x}")
            t.check(r["handle"] == h and (r["flags"] & 1) == inc, f"{tag}: record handle/INCOMP != CQE")
            t.check(r["lsize"] == n, f"{tag}: LSIZE {r['lsize']}")
            t.check(r["psize"] == up4k(r["csize"]), f"{tag}: psize {r['psize']} for csize {r['csize']}")
            t.check(r["adler"] == zlib.adler32(rec), f"{tag}: Adler-32 {r['adler']:08x} != {zlib.adler32(rec):08x}")
            t.check(r["status"] == 0, f"{tag}: record status {r['status']:x}")
            nlb = r["psize"] // 4096
            slba = alloc(nlb)
            d.put(GUARD, b"\xee" * 4096)
            d.rw(True, slba + nlb, 1, off=GUARD, lst=PRPL_A)
            st = d.commit(h, slba, nlb)
            if not t.check(st == 0, f"{tag}: 84h commit {stname(st)}"):
                d.commit(h, 0, 1, discard=True)
                continue
            d.put(BUF_B, b"\xa5" * (nlb + 1) * 4096)
            st = d.rw(False, slba, nlb + 1)
            disk = d.get(BUF_B, (nlb + 1) * 4096)
            stored = disk[:r["psize"]]
            t.check(st == 0, f"{tag}: read back {stname(st)}")
            t.check(disk[r["psize"]:] == b"\xee" * 4096, f"{tag}: block after the extent overwritten")
            t.check(fletcher4(stored) == r["fl"], f"{tag}: Fletcher-4 of the committed bytes")
            if inc:
                t.check(r["csize"] == n and stored[:n] == rec, f"{tag}: incompressible record not stored raw")
                t.check(kind == "random", f"{tag}: unexpectedly incompressible")
            else:
                t.check(r["csize"] <= n - n // 8, f"{tag}: kept compressed above the 12.5% rule")
                try:
                    t.check(zlib.decompress(stored[:r["csize"]]) == rec, f"{tag}: zlib.decompress(committed) != record")
                except zlib.error as e:
                    t.check(False, f"{tag}: Python zlib rejects the committed stream: {e}")
                t.check(not any(stored[r["csize"]:]), f"{tag}: padding not zero")
                st, out = d.dread(slba, nlb, r["csize"], n)
                t.check(st == 0 and out == rec, f"{tag}: 86h Decompress-Read st={stname(st)} match={out == rec}")
                ext.append((rec, slba, nlb, r))
            print(f"{kind:>8} {n:>7} {r['csize']:>7} {r['psize']:>7} {n / r['csize']:>5.2f}x "
                  f"{len(zlib.compress(rec, 6)):>9}  {'INCOMP' if inc else 'ok'}{'' if t.bad == bad0 else '  <-- FAIL'}")

    # ---- threshold ----
    print("\nthreshold percent T (128 KB text), at the boundary of csize * 100 <= LSIZE * T:")
    rec = gen("text", REC, rnd)
    st, h, inc, r = d.c2s(rec)
    d.commit(h, 0, 1, discard=True)
    t_ok = -(-r["csize"] * 100 // REC)                    # smallest T that keeps it compressed
    for thr, want in ((100, 0), (t_ok, 0), (t_ok - 1, 1), (1, 1)):
        if thr < 1:
            continue
        st, h, inc, r2 = d.c2s(rec, thr=thr)
        ok = t.check(st == 0 and inc == want and r2["csize"] == (REC if want else r["csize"]),
                     f"T={thr}: st={stname(st)} INCOMP={inc}, expected {want}")
        if st == 0:
            t.check(d.commit(h, 0, 1, discard=True) == 0, f"T={thr}: discard")
        print(f"  T={thr:>3}%: csize {r2['csize']:>6}  {'INCOMP' if inc else 'compressed'}  {'ok' if ok else 'FAIL'}")

    # ---- several staged, committed in reverse order ----
    print("\nstreaming: 8 records staged, then committed in reverse order")
    staged = []
    for i in range(8):
        rec = gen(("text", "runs", "random", "zeros")[i % 4], REC, rnd)
        st, h, inc, r = d.c2s(rec, res=RES + 64 * i)
        t.check(st == 0, f"stage #{i}: {stname(st)}")
        staged.append((rec, h, inc, r))
    hs = [s[1] for s in staged]
    t.check(len(set(hs)) == 8, f"handles not distinct: {hs}")
    placed = []
    for rec, h, inc, r in reversed(staged):
        nlb = r["psize"] // 4096
        slba = alloc(nlb)
        t.check(d.commit(h, slba, nlb) == 0, f"commit handle {h}")
        placed.append((rec, slba, nlb, inc, r))
    good = 0
    for rec, slba, nlb, inc, r in placed:
        d.rw(False, slba, nlb)
        stored = d.get(BUF_B, nlb * 4096)
        ok = fletcher4(stored) == r["fl"]
        ok = ok and (stored[:len(rec)] == rec if inc else zlib.decompress(stored[:r["csize"]]) == rec)
        good += t.check(ok, f"reverse-commit extent at LBA {slba}: contents")
    print(f"  handles {hs}: {good}/8 extents verified")

    # ---- errors ----
    print("\nerror handling:")

    def expect(what, st, want):
        ok = t.check(st == want, f"{what}: {stname(st)}, expected {stname(want)}")
        print(f"  {what:<44} -> {stname(st)}{'' if ok else '  <-- FAIL'}")

    expect("81h LSIZE 0", d.c2s(b"", lsize=0)[0], ST_INVALID_FIELD)
    expect("81h LSIZE 1000 (not a 512 multiple)", d.c2s(b"y" * 1024, lsize=1000)[0], ST_INVALID_FIELD)
    expect("81h LSIZE 128 KB + 512", d.c2s(gen("text", REC + 512, rnd))[0], ST_INVALID_FIELD)
    rec = gen("text", 65536, rnd)
    st, h, inc, r = d.c2s(rec)
    nlb = r["psize"] // 4096
    expect("84h NLB one too large", d.commit(h, alloc(nlb + 1), nlb + 1), ST_INVALID_FIELD)
    slba = alloc(nlb)
    expect("84h correct NLB after that (slot kept)", d.commit(h, slba, nlb), 0)
    expect("84h same handle again (already freed)", d.commit(h, slba, nlb), ST_HANDLE)
    expect(f"84h handle {nslot} (>= NSLOT)", d.commit(nslot, slba, nlb), ST_HANDLE)
    d.rw(False, slba, nlb)
    t.check(zlib.decompress(d.get(BUF_B, r["csize"])) == rec, "extent after NLB retry")
    st, h, inc, r = d.c2s(rec)
    nlb = r["psize"] // 4096
    nsze = d.nsze or 245760
    expect("84h past the end of the namespace", d.commit(h, nsze - 1, nlb), ST_LBA_RANGE)
    expect("84h DISCARD", d.commit(h, 0, 1, discard=True), 0)
    expect("84h after DISCARD", d.commit(h, slba, nlb), ST_HANDLE)

    hs, st = [], 0
    for i in range(nslot + 1):
        st, h, inc, r = d.c2s(gen("text", 4096, rnd), res=RES + 64 * i)
        if st:
            break
        hs.append(h)
    t.check(len(hs) == nslot and len(set(hs)) == nslot, f"staged {len(hs)} records before refusal, NSLOT {nslot}")
    expect(f"81h with all {len(hs)} slots staged", st, ST_FULL)
    dis = sum(d.commit(h, 0, 1, discard=True) == 0 for h in hs)
    t.check(dis == len(hs), f"discarded {dis}/{len(hs)}")
    st, h, *_ = d.c2s(gen("text", 4096, rnd))
    expect("81h after discarding them all", st, 0)
    if st == 0:
        d.commit(h, 0, 1, discard=True)

    if ext:
        rec, slba, nlb, r = max(ext, key=lambda e: len(e[0]))
        rr = bytearray(struct.pack("<HHIII4QIHH8x", r["handle"], r["flags"], r["lsize"], r["csize"],
                                            r["psize"], *r["fl"], r["adler"], r["lid"], r["status"]))
        d.put(RES2, bytes(rr))
        st, out = d.dread(slba, nlb, r["csize"], len(rec), res=RES2)
        expect("86h with a rebuilt Result Record", st, 0)
        t.check(out == rec, "86h with rebuilt record: data")
        rr[0x10] ^= 1
        d.put(RES2, bytes(rr))
        st, out = d.dread(slba, nlb, r["csize"], len(rec), res=RES2)
        expect("86h with a wrong Fletcher-4", st, ST_CKSUM)
        st, out = d.dread(slba, nlb, r["csize"], len(rec), res=RES2, nochk=True)
        expect("86h same, NOCHK", st, 0)
        t.check(out == rec, "86h NOCHK: data")
        st, out = d.dread(slba, nlb, r["csize"], len(rec) - 4096, res=RES2, nochk=True)
        expect("86h ULEN 4 KB short", st, ST_LEN)

    # ---- rate ----
    rec = gen("text", REC, rnd)
    N = 200
    t0 = time.perf_counter()
    for i in range(N):
        st, h, inc, r = d.c2s(rec)
        d.commit(h, TEST_LBA + 40000, r["psize"] // 4096)
    t1 = time.perf_counter()
    nlb = r["psize"] // 4096
    for i in range(N):
        st, out = d.dread(TEST_LBA + 40000, nlb, r["csize"], REC)
    t2 = time.perf_counter()
    t.check(out == rec, "rate loop: last decompress-read")
    mb = N * REC / 1e6
    print(f"\nrate, 128 KB text records ({REC / r['csize']:.2f}x), one command at a time, polled from Python:")
    print(f"  stage + commit  {mb / (t1 - t0):7.1f} MB/s  ({(t1 - t0) / N * 1e6:.0f} us/record)")
    print(f"  decompress-read {mb / (t2 - t1):7.1f} MB/s  ({(t2 - t1) / N * 1e6:.0f} us/record)")


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    bdf = args[0] if args else find_board()
    if os.geteuid() != 0:
        raise SystemExit("run as root")
    orig = drv(bdf)
    print(f"board {bdf} (driver {orig}) -> vfio-pci")
    to_vfio(bdf)
    v = d = None
    t = T()
    try:
        v = Vfio(bdf)
        d = Nvme(v)
        print(f"controller up: {d.model} fw {d.fw} MDTS {d.mdts} NSZE {d.nsze}\n")
        tests(d, t, random.Random(2))
    except Exception as e:
        t.check(False, f"aborted: {type(e).__name__}: {e}")
    finally:
        if d:
            try:
                d.set_c0(0)
                d.shutdown()
            except Exception as e:
                print(f"cleanup: {e}")
        if v:
            v.close()
        if "--keep-vfio" not in sys.argv:
            to_nvme(bdf)
            print(f"\nboard handed back to {drv(bdf)}")
    print(f"\n{t.ok} checks passed, {t.bad} failed -> {'PASS' if t.bad == 0 else 'FAIL'}")
    return 0 if t.bad == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
