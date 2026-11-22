#!/usr/bin/env python3
"""Generate zlib test vectors for the system testbench (sim/zvec.hex).

Format (bytes, one hex byte per line): for each vector
  ulen (4 bytes LE), clen (4 bytes LE), ulen raw bytes, clen zlib bytes
terminated by ulen = 0. Streams come from Python's zlib, so they exercise stored,
fixed and dynamic Huffman blocks exactly as written by software gzip (ZFS fallback).
"""
import os, random, struct, sys, zlib

def text(n, r):
    w = [b"zfs", b"record", b"block", b"nvme", b"fpga", b"the", b"of", b"data", b"\n", b"compress"]
    out = bytearray()
    while len(out) < n:
        out += r.choice(w) + b" "
    return bytes(out[:n])

def main(path, big):
    r = random.Random(42)
    vecs = []
    for n, lvl, strat in [(4096, 6, 0), (4096, 0, 0), (12288, 1, 0), (32768, 9, 0),
                          (65536, 6, zlib.Z_FIXED), (65536, 6, zlib.Z_RLE), (131072, 6, 0)]:
        d = text(n, r)
        c = zlib.compressobj(lvl, zlib.DEFLATED, 15, 9, strat)
        vecs.append((d, c.compress(d) + c.flush()))
    d = bytes(r.getrandbits(8) for _ in range(8192))      # incompressible, stored at level 0
    vecs.append((d, zlib.compress(d, 0)))
    if big:
        d = text(1 << 20, r)
        vecs.append((d, zlib.compress(d, 6)))
    out = bytearray()
    for d, z in vecs:
        out += struct.pack("<II", len(d), len(z)) + d + z
    out += struct.pack("<II", 0, 0)
    with open(path, "w") as f:
        f.write("\n".join("%02x" % b for b in out) + "\n")
    print(f"{path}: {len(vecs)} vectors, {len(out)} bytes")

if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "zvec.hex", "--big" in sys.argv)
