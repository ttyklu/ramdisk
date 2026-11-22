#!/usr/bin/env python3
"""Unit regression for rtl/deflate_enc.sv and rtl/inflate.sv against Python's zlib.

  scripts/codec_test.py            build once with Verilator, run every case
  scripts/codec_test.py -k text    only cases whose name contains 'text'
  scripts/codec_test.py --big      also 1 MB records

deflate cases: the RTL output must be a valid zlib stream that zlib.decompress()
turns back into the input, with the right Adler-32.
inflate cases: zlib streams from Python (levels 0-9, fixed/RLE/Huffman-only strategies,
multi-block) must decode to the original; corrupted streams must report an error.
round cases: deflate -> inflate inside the testbench.
"""
import argparse, os, random, subprocess, sys, zlib

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORK = os.path.join(ROOT, "sim", "codec_work")
EXE = os.path.join(WORK, "obj", "Vtb_codec")
ERR = {0: "ok", 1: "hdr", 2: "btype", 3: "stored", 4: "codes", 5: "sym", 6: "dist", 7: "trunc", 8: "adler"}


def build():
    os.makedirs(WORK, exist_ok=True)
    cmd = ["verilator", "--binary", "--timing", "-sv", "-j", str(os.cpu_count()),
           "-Wno-fatal", "-Wno-lint", "-Wno-style", "-Wno-TIMESCALEMOD",
           os.path.join(ROOT, "rtl/ram_1r1w.sv"), os.path.join(ROOT, "rtl/deflate_enc.sv"),
           os.path.join(ROOT, "rtl/inflate.sv"), os.path.join(ROOT, "tb/unit/tb_codec.sv"),
           "--top-module", "tb_codec", "--Mdir", os.path.join(WORK, "obj"), "-o", "Vtb_codec"]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode:
        print(r.stdout[-3000:], r.stderr[-3000:])
        sys.exit("build failed")


def wr(path, data):
    with open(path, "w") as f:
        f.write("\n".join("%02x" % b for b in data) + "\n")


def rd(path):
    with open(path) as f:
        return bytes(int(x, 16) for x in f.read().split())


def run(mode, data, seed, stall=20):
    fin = os.path.join(WORK, "in.hex")
    fout = os.path.join(WORK, "out.hex")
    wr(fin, data if data else b"\0")
    if os.path.exists(fout):
        os.remove(fout)
    r = subprocess.run([EXE, f"+mode={mode}", f"+in={fin}", f"+n={len(data)}", f"+out={fout}",
                        f"+stall={stall}", f"+verilator+seed+{seed}"],
                       capture_output=True, text=True, cwd=WORK)
    res = [l for l in r.stdout.splitlines() if l.startswith("RESULT")]
    if not res:
        return None, None, r.stdout[-2000:]
    kv = dict(x.split("=") for x in res[0].split()[2:] if "=" in x)
    out = rd(fout) if os.path.exists(fout) else b""
    return kv, out, res[0]


# ---------------- test data ----------------
def gen(kind, n, rnd):
    if kind == "zeros":
        return bytes(n)
    if kind == "random":
        return bytes(rnd.getrandbits(8) for _ in range(n))
    if kind == "text":
        words = [b"the", b"quick", b"brown", b"fox", b"jumps", b"over", b"lazy", b"dog", b"ZFS",
                 b"record", b"compress", b"NVMe", b"\n", b"block", b"data"]
        out = bytearray()
        while len(out) < n:
            out += rnd.choice(words) + b" "
        return bytes(out[:n])
    if kind == "runs":
        out = bytearray()
        while len(out) < n:
            out += bytes([rnd.getrandbits(8)]) * rnd.randint(1, 300)
        return bytes(out[:n])
    if kind == "mixed":
        out = bytearray()
        while len(out) < n:
            k = rnd.choice(["text", "random", "runs", "zeros"])
            out += gen(k, rnd.randint(1, 5000), rnd)
        return bytes(out[:n])
    if kind == "far":  # repeats at distances near 32 KB
        base = gen("random", 33000, rnd)
        return (base + base[:n])[:n]
    raise ValueError(kind)


def cases(big):
    rnd = random.Random(1234)
    sizes = [1, 2, 3, 4, 5, 257, 258, 259, 4096, 70000, 131072]
    if big:
        sizes += [1 << 20]
    out = []
    for kind in ["zeros", "random", "text", "runs", "mixed", "far"]:
        for n in sizes:
            if kind == "far" and n < 40000:
                continue
            out.append((f"{kind}-{n}", gen(kind, n, rnd)))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-k", default="")
    ap.add_argument("--big", action="store_true")
    ap.add_argument("--nobuild", action="store_true")
    a = ap.parse_args()
    if not a.nobuild:
        build()
    fails = 0
    total = 0
    seed = 1

    def report(name, ok, msg):
        nonlocal fails, total
        total += 1
        if not ok:
            fails += 1
        print(("PASS " if ok else "FAIL ") + name + ("" if ok else "  " + msg))

    data_cases = [c for c in cases(a.big) if a.k in c[0]]

    # deflate: RTL output must decompress with zlib
    for name, data in data_cases:
        seed += 1
        kv, out, line = run("deflate", data, seed)
        if kv is None:
            report("deflate " + name, False, line)
            continue
        try:
            back = zlib.decompress(out)
            ok = back == data and int(kv["adler"], 16) == zlib.adler32(data) and int(kv["csize"]) == len(out)
            msg = f"csize={len(out)} ratio={len(out) / max(1, len(data)):.3f}"
        except zlib.error as e:
            ok, msg = False, f"zlib: {e}"
        report(f"deflate {name:14s} {len(data):8d} -> {len(out):8d}", ok, msg)

    # inflate: zlib streams from Python
    strategies = [("L0", dict(level=0)), ("L1", dict(level=1)), ("L6", dict(level=6)), ("L9", dict(level=9)),
                  ("fixed", dict(level=6, strategy=zlib.Z_FIXED)), ("rle", dict(level=6, strategy=zlib.Z_RLE)),
                  ("huff", dict(level=6, strategy=zlib.Z_HUFFMAN_ONLY))]
    for name, data in data_cases:
        for sname, kw in strategies:
            if len(data) > 70000 and sname not in ("L6", "L0"):
                continue
            seed += 1
            c = zlib.compressobj(kw.get("level", 6), zlib.DEFLATED, 15, 9, kw.get("strategy", 0))
            z = c.compress(data) + c.flush()
            kv, out, line = run("inflate", z, seed)
            if kv is None:
                report(f"inflate {name} {sname}", False, line)
                continue
            ok = kv["err"] == "0" and out == data and int(kv["consumed"]) == len(z)
            report(f"inflate {name:14s} {sname:5s}", ok,
                   f"err={ERR.get(int(kv['err']))} produced={kv['produced']}/{len(data)} consumed={kv['consumed']}/{len(z)}")

    # inflate error cases
    rnd = random.Random(7)
    data = gen("text", 5000, rnd)
    z = zlib.compress(data, 6)
    bad = {
        "bad-adler": z[:-1] + bytes([z[-1] ^ 1]),
        "truncated": z[: len(z) // 2],
        "bad-header": bytes([0x78, 0x02]) + z[2:],
        "btype3": bytes([0x78, 0x01, 0x07]) + z[3:],
        "stored-nlen": bytes([0x78, 0x01, 0x01, 0x05, 0x00, 0x00, 0x00]) + b"hello" + bytes(4),
    }
    for bname, zb in bad.items():
        seed += 1
        kv, out, line = run("inflate", zb, seed)
        ok = kv is not None and kv["err"] != "0"
        report(f"inflate-error {bname:12s}", ok, line if kv is None else f"err={ERR.get(int(kv['err']))}")
    # trailing bytes after the stream are not consumed
    seed += 1
    kv, out, line = run("inflate", z + bytes(100), seed)
    ok = kv is not None and kv["err"] == "0" and int(kv["consumed"]) == len(z) and out == data
    report("inflate trailing-padding", ok, line)

    # round trip inside the testbench
    for name, data in data_cases:
        if len(data) > 131072:
            continue
        seed += 1
        kv, out, line = run("round", data, seed)
        ok = kv is not None and kv["err"] == "0" and kv["errors"] == "0" and int(kv["produced"]) == len(data)
        report(f"round   {name}", ok, line)

    print(f"{total - fails}/{total} passed")
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
