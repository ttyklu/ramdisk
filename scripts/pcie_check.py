#!/usr/bin/env python3
"""pcie_check.py - exercise the ZC706 NVMe RAM disk over PCIe and verify every byte.

    sudo python3 scripts/pcie_check.py [/dev/nvme1n1] [--mib N]

DESTRUCTIVE: overwrites the first N MiB of the namespace (default: all of it).

  1. Sequential O_DIRECT write of seeded pseudo-random data (1 MiB I/Os), timed
  2. Sequential O_DIRECT read-back, timed, compared byte-for-byte
  3. 4000 random-offset, random-size (4 KiB..256 KiB) write/read/compare round-trips
  4. Kernel log and PCI status/AER checked before and after for new errors
"""
import mmap, os, random, re, subprocess, sys, time

dev = next((a for a in sys.argv[1:] if a.startswith("/dev/")), "/dev/nvme1n1")
blk = os.path.basename(dev)
size = int(open(f"/sys/block/{blk}/size").read()) * 512
if "--mib" in sys.argv:
    size = min(size, int(sys.argv[sys.argv.index("--mib") + 1]) << 20)
CHUNK = 1 << 20
SEED = 0x2C706

def pdev():
    p = os.path.realpath(f"/sys/block/{blk}/device/device")
    return os.path.basename(p)

def pci_snapshot(bdf):
    out = subprocess.run(["lspci", "-s", bdf, "-vv"], capture_output=True, text=True).stdout
    keep = [l.strip() for l in out.splitlines()
            if re.search(r"Status:|LnkSta:|DevSta:|UESta:|CESta:", l)]
    return keep

def klog_lines():
    return subprocess.run(["dmesg"], capture_output=True, text=True).stdout.splitlines()

def pattern(i, n):
    return random.Random(SEED * 1_000_003 + i).randbytes(n)

def abuf(n):
    return mmap.mmap(-1, n)          # page-aligned, as O_DIRECT needs

bdf = pdev()
print(f"device {dev}  pci {bdf}  test size {size >> 20} MiB")
pre_pci, pre_log = pci_snapshot(bdf), klog_lines()
for l in pre_pci: print("  before:", l)

fd = os.open(dev, os.O_RDWR | os.O_DIRECT | os.O_SYNC)
buf = abuf(CHUNK)
nchunks = size // CHUNK
fails = 0

# 1. sequential write
t = time.perf_counter()
for i in range(nchunks):
    buf.seek(0); buf.write(pattern(i, CHUNK))
    os.pwritev(fd, [buf], i * CHUNK)
tw = time.perf_counter() - t
print(f"seq write : {nchunks} MiB in {tw:.2f}s = {nchunks / tw:.1f} MiB/s (QD1, incl. pattern gen)")

# 2. sequential read + verify (time the reads only)
tr = 0.0
for i in range(nchunks):
    t = time.perf_counter()
    os.preadv(fd, [buf], i * CHUNK)
    tr += time.perf_counter() - t
    if buf[:] != pattern(i, CHUNK):
        exp = pattern(i, CHUNK)
        off = next(k for k in range(CHUNK) if buf[k] != exp[k])
        fails += 1
        if fails <= 10:
            print(f"  MISMATCH chunk {i} first bad byte at 0x{i * CHUNK + off:x}: "
                  f"got {buf[off:off+16].hex()} exp {exp[off:off+16].hex()}")
print(f"seq read  : {nchunks} MiB in {tr:.2f}s = {nchunks / tr:.1f} MiB/s (QD1)  "
      f"mismatched chunks: {fails}")

# 3. random offsets / sizes
rng = random.Random(SEED)
big = abuf(256 << 10)
rfails = 0
for n in range(4000):
    ln = rng.randrange(1, 65) * 4096
    off = rng.randrange(0, (size - ln) // 4096 + 1) * 4096
    data = rng.randbytes(ln)
    big.seek(0); big.write(data)
    os.pwritev(fd, [memoryview(big)[:ln]], off)
    big.seek(0); big.write(b"\xa5" * ln)
    os.preadv(fd, [memoryview(big)[:ln]], off)
    if big[:ln] != data:
        rfails += 1
        if rfails <= 10:
            print(f"  MISMATCH random #{n}: off 0x{off:x} len {ln}")
print(f"random rw : 4000 round-trips, mismatches: {rfails}")
os.close(fd)

# 4. errors
post_pci = pci_snapshot(bdf)
new_log = klog_lines()[len(pre_log):]
bad = [l for l in new_log if re.search(r"nvme|AER|pcie|timeout|reset|error", l, re.I)]
for l in post_pci: print("  after :", l)
print("new kernel messages:", *(bad or ["(none)"]), sep="\n  ")
ok = fails == 0 and rfails == 0 and not bad and pre_pci == post_pci
print("RESULT:", "PASS" if ok else "FAIL")
sys.exit(0 if ok else 1)
