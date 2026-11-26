# ZC706 NVMe RAM Disk (with hardware compression)

A hardware NVMe controller for the ZC706 that serves a namespace out of DDR3. No ARM
firmware is involved: admin and I/O commands are handled entirely in the FPGA fabric.
Hardware gzip (zlib) compression for ZFS runs through vendor commands: a streaming
two-phase write (Compress-to-Staging, then Commit) and a single-phase Decompress-Read, plus a
subset of the draft SNIA FFE-Send / FFE-Receive commands. The namespace is RAW by default.
Interface: [docs/compression_spec.md](docs/compression_spec.md).

**Status:** the self-checking testbench passes in both Vivado xsim 2026.1 and Verilator 5.032,
for 128 KB records (20/20 random seeds) and the optional 1 MB build. The deflate / inflate
engines also pass 494 unit checks against Python's zlib. Out-of-context place and route on the
ZC706 part (xc7z045ffg900-2) meets timing at 125 MHz for both builds (WNS +0.15 ns at 128 KB,
+0.30 ns at 1 MB).

**On hardware** (ZC706, `fpga/zc706/`, DDR3-800): the board enumerates under the Linux 7.0
`nvme` driver as `nvme1n1` (960 MiB, 4 KB blocks) and trains at Gen2 x4. The whole namespace
reads back byte-for-byte with no PCIe errors (`scripts/pcie_check.py`), and the two-phase
compressed write, Decompress-Read and their error cases pass 349 of 349 checks
(`scripts/staging_hwtest.py`). See [Testing on the board](#testing-on-the-board).

## What is implemented

| Area | Details |
| --- | --- |
| PCIe | 7 Series Integrated Block (`pcie_7x`) 128-bit AXI-Stream, Gen2 x4, 125 MHz `user_clk`; RX straddling handled (`rx128_to_64`, `tx64_to_128` adapt to the internal 64-bit TLP beats) |
| BAR0 (16 KB) | CAP / VS (2.4) / INTMS / INTMC / CC / CSTS / AQA / ASQ / ACQ, doorbells at 0x1000 (DSTRD = 0), MSI-X table at 0x2000, PBA at 0x3000 |
| Queues | Admin + 4 I/O queue pairs (`NQ`), up to 4096 entries each, physically contiguous only |
| Admin commands | Identify (CNS 0, 1, 2; any other CNS returns 4 KB of zeros), Get Log Page (SMART has temperature/spare, others zero), Set/Get Features (Number of Queues answered, others accepted), Create/Delete I/O SQ/CQ, Abort, Asynchronous Event Request (held, never completed) |
| I/O commands | Read, Write, Flush. 4 KB LBAs, one namespace, MDTS = 128 KB (32 blocks) |
| PRPs | PRP1 at any DW offset, PRP2, PRP lists including chained list pages |
| DMA reads | Up to 16 outstanding MRd (128 B each, never crossing 128 B); completions may arrive out of order across tags |
| DMA writes | MWr up to 128 B, never crossing 128 B; 3DW headers below 4 GB, 4DW above |
| Interrupts | MSI-X, one message per completion, per-vector mask and pending bits |
| DDR | AXI4 master, 64-bit, 128 B INCR bursts; namespace starts at `DDR_BASE`, 64 MB staging ring after it |
| Compression | Set/Get Features C0h (CEN, record size; RAW default). Compress-to-Staging 81h, Commit 84h, Decompress-Read 86h, FFE-Send 89h (82h compress, 83h decompress), FFE-Receive 8Ah |
| Codec | zlib streams. Compress: LZ77 (32 KB window, 4096-entry hash) + fixed Huffman. Decompress: stored, fixed and dynamic blocks, Adler-32 checked |
| Checksums | ZFS Fletcher-4 of the stored bytes (zero padded to 4 KB), Adler-32 of the record |
| Records | 128 KB (`REC_LOG2 = 17`, default) up to 1 MB (`REC_LOG2 = 20`); MDTS = 2 x record; 128 slots at 128 KB, 16 at 1 MB |
| BRAM | Estimated 45 RAMB36 of 545 (not yet synthesized): 64 KB x 2 deflate history, deflate hash table, 32 KB inflate history, slot records. Staging slots live in DDR |

## Architecture

```
pcie_7x RX ─► tlp_rx ─┬─► nvme_regs (BAR0) ──► C port ─┐
                      │      │ doorbells, CC, MSI-X    │
                      │      ▼                          │
                      │   nvme_ctrl (command FSM) ◄─────┼── arbitration, admin, PRP walk, CQE, MSI-X
                      │      │ jobs                     │
                      └─► dma_rd (tags + reorder ring) ─┼─► R port (MRd)
                             │ DW stream                ▼
                             ├─► SQE / PRP list      tlp_tx ─► pcie_7x TX
                             └─► ddr_wr ─► AXI W        ▲
                   AXI R ─► ddr_rd ─► TX DW FIFO ─► mwr_eng ─► W port (MWr)
                     Identify / log / CQE / MSI data ─┘
```

One command is processed at a time. Within a command, DMA requests, DDR bursts and TLPs
are pipelined.

## Files

| File | Purpose |
| --- | --- |
| `rtl/nvme_top.sv` | Top level: pcie_7x AXI-Stream + cfg ports, AXI4 master to DDR |
| `rtl/nvme_ctrl.sv` | Command engine (enable/disable, arbitration, admin, I/O, PRP, CQE, MSI-X) |
| `rtl/nvme_regs.sv` | BAR0 registers, doorbells, MSI-X table/PBA, register-read completions |
| `rtl/tlp_rx.sv`, `rtl/tlp_tx.sv` | TLP parse / build for the 7-series 64-bit interface |
| `rtl/dma_rd.sv` | Host-memory reads with tag reorder ring |
| `rtl/mwr_eng.sv` | Splits host writes into MWr TLPs |
| `rtl/ddr_rd.sv`, `rtl/ddr_wr.sv` | AXI4 burst engines for the DDR |
| `rtl/dw_fifo.sv`, `rtl/dw_ram2.sv` | 2-DW-per-cycle FIFO / RAM (LUTRAM) |
| `rtl/xform.sv` | DDR -> COPY / DEFLATE / INFLATE -> DDR or TX FIFO, with Fletcher-4 in and out |
| `rtl/deflate_enc.sv`, `rtl/inflate.sv` | zlib compressor (LZ77 + Huffman) and decompressor |
| `rtl/fletcher4.sv`, `rtl/ram_1r1w.sv` | ZFS Fletcher-4; block RAM primitive |
| `tb/tb_comp.svh` | Compression tests (included by `tb_top.sv`) |
| `tb/unit/tb_codec.sv`, `scripts/codec_test.py` | Codec unit tests against Python zlib |
| `scripts/gen_zvec.py` | zlib streams from software for the system test |
| `docs/compression_spec.md` | Opcodes, dword layouts, CRB, status codes, slots |
| `rtl/nvme_identify.sv` | Identify data, generated by `scripts/gen_identify.py` |
| `tb/tb_top.sv` | Root-complex model + NVMe host driver, self-checking |
| `tb/axi_mem_model.sv` | AXI4 DDR model with random stalls |
| `sim/run_xsim.sh` | Compile and run with Vivado xsim |
| `fpga/zc706/` | Board build: `zc706_top.sv`, `zc706.xdc`, `gen_ip.tcl` (pcie_7x + MIG IP), `build.tcl` (bitstream) |
| `scripts/pcie_check.py` | Hardware: full-namespace write / read-back / compare through the Linux block device |
| `scripts/ffe_hwtest.py` | Hardware: FFE-Send / FFE-Receive compress and decompress through Linux NVMe passthrough |
| `scripts/staging_hwtest.py` | Hardware: two-phase write (81h + 84h) and Decompress-Read (86h) through VFIO |
| `scripts/vs_passthru_check.py` | Hardware: which compression vendor commands work through the Linux `nvme` passthrough ioctl |
| `scripts/md2html.py` | Renders `README.md` and `docs/compression_spec.md` to HTML |

## Running the simulation

```bash
source /path/to/Xilinx/2026.1/Vivado/settings64.sh
cd ~/ramdisk/sim
./run_xsim.sh          # batch; prints TEST PASSED / TEST FAILED
./run_xsim.sh waves    # also records waves.wdb (open with: xsim waves.wdb -gui)
./run_xsim.sh gui      # interactive
```

Or with Verilator (5.x, uses `--timing`):

```bash
cd ~/ramdisk/sim
./run_verilator.sh            # build + run; prints TEST PASSED / TEST FAILED
./run_verilator.sh waves      # also dumps waves.vcd (open with: gtkwave waves.vcd)
./run_verilator.sh seeds 50   # run 50 random seeds
SEED=7 ./run_verilator.sh     # one specific seed
REC=20 ./run_verilator.sh     # build with 1 MB records (also: REC=20 ./run_xsim.sh)
python3 ../scripts/codec_test.py [--big]   # deflate / inflate unit tests against zlib
```

The testbench brings the controller up the way the Linux `nvme` driver does (CAP/VS reads,
admin queue, CC.EN, Identify, Set Features, create I/O queues), then runs error cases
and about 90 reads/writes with random sizes, PRP1 offsets, PRP2 and chained PRP lists
on two I/O queues. Every read is compared against a disk model. Completions to the
controller's reads are split at 64 B boundaries and interleaved across tags; the TX
interface and the DDR model apply random backpressure.

## FPGA build (out of context)

```bash
cd ~/ramdisk/fpga
vivado -mode batch -source ooc_build.tcl -tclargs 17 route 8.0    # 128 KB build, 125 MHz
vivado -mode batch -source ooc_build.tcl -tclargs 20 route 8.0    # 1 MB build
```

Reports land in `fpga/reports_rec<N>/`. `fpga/quick_mod.tcl` runs one module for timing work.

| Build | LUTs | Flip-flops | RAMB36 | WNS at 8 ns |
| --- | --- | --- | --- | --- |
| 128 KB records | 28,756 (13%) | 16,596 (4%) | 50 (9%) | +0.154 ns |
| 1 MB records | 33,068 (15%) | 37,937 (9%) | 50 (9%) | +0.297 ns |

## Testing on the board

Build and boot: `cd fpga/zc706 && vivado -mode batch -source build.tcl`, then
`./boot/make_boot.sh` and copy `boot/work/BOOT.bin` to the SD card as `BOOT.BIN`. The board
boots from SD. The SODIMM
(MT8JTF12864HZ-1G6G1) only calibrates at DDR3-800, which is what the build uses.

Check that the host sees it:

```bash
lspci -nn -d 10ee:                              # Non-Volatile memory controller [10ee:7024]
cat /sys/bus/pci/devices/0000:08:00.0/current_link_{speed,width}   # 5.0 GT/s PCIe, 4
lsblk -o NAME,SIZE,MODEL | grep ZC706           # nvme1n1  960M ZC706 NVMe RAM Disk
```

If the controller does not come up, reset the function and rebind it:

```bash
echo 1 | sudo tee /sys/bus/pci/devices/0000:08:00.0/reset
echo 0000:08:00.0 | sudo tee /sys/bus/pci/drivers/nvme/bind
```

The scripts need root, and `sudo` needs a terminal to ask for the password (it will not run
from a `!` command in Claude Code). Run them in a normal terminal.

| Script | What it does | Destructive |
| --- | --- | --- |
| `sudo python3 scripts/pcie_check.py [/dev/nvme1n1] [--mib N]` | O_DIRECT sequential write of a seeded pattern in 1 MiB I/Os, timed read-back with byte compare, then 4000 random-offset, random-size (4-256 KB) round-trips. Compares PCI status / DevSta / LnkSta and the kernel log before and after | Whole namespace, or the first N MiB |
| `sudo python3 scripts/ffe_hwtest.py [/dev/nvme1]` | FFE-Send / FFE-Receive (82h compress, 83h decompress) against Python zlib, LBA destination and source, error codes, rates | LBAs >= 200000 |
| `sudo python3 scripts/staging_hwtest.py [0000:08:00.0]` | Two-phase write and Decompress-Read; see [docs/compression_spec.md](docs/compression_spec.md#testing-on-hardware) | LBAs >= 200000 |
| `sudo python3 scripts/vs_passthru_check.py [/dev/nvme1]` | Vendor commands through the Linux driver; see [docs/compression_spec.md](docs/compression_spec.md#what-works-through-the-linux-nvme-driver) | LBAs >= 230000 |

Each prints `PASS` or `FAIL` at the end and exits non-zero on failure.

Results on 2026-11-26 (host: Intel root port 8086:7a30, Linux 7.0.0-44):

| Test | Result |
| --- | --- |
| Link | 5.0 GT/s x4 (the maximum of both ends) |
| `pcie_check.py`, 960 MiB | PASS: 0 mismatched chunks, 0 of 4000 random round-trips, no new kernel messages, PCI status unchanged |
| Sequential write, QD1, 1 MiB | 304 MiB/s (includes generating the pattern in Python) |
| Sequential read, QD1, 1 MiB | 701 MiB/s |
| `staging_hwtest.py` | PASS: 349 of 349 checks |
| `ffe_hwtest.py` | PASS: 245 of 245 checks (largest record 124 KB, see below) |
| `vs_passthru_check.py` | PASS: 20 of 20 checks; 81h refused by the kernel (EINVAL), as expected |
| FFE compress / decompress, 124 KB text | 38.2 / 26.0 MB/s (FFE-Send + FFE-Receive, Python ioctl) |
| Stage + commit, 128 KB text | 50.5 MB/s (2.6 ms per record, one command at a time, polled from Python) |
| Decompress-Read, 128 KB text | 31.1 MB/s (4.2 ms per record) |

## PCIe core settings

`pcie_7x` settings the RTL assumes (used by `fpga/zc706/gen_ip.tcl`):

- Gen2 (5.0 GT/s) x4, 128-bit AXI-Stream interface, 125 MHz
- BAR0: 64-bit memory BAR, 16 KB, not prefetchable; no other BARs
- Class code `0x010802` (mass storage / NVM / NVMe)
- MSI-X capability: table size 5, table in BAR0 at offset 0x2000, PBA in BAR0 at 0x3000; MSI off
- Vendor/device IDs of your choice (Identify reports VID/SSVID 0x10EE)

Connect `user_clk_out` / `user_reset_out` to `clk` / `rst`, `cfg_bus_number`,
`cfg_device_number`, `cfg_function_number`, `cfg_interrupt_msixenable` and
`cfg_interrupt_msixfm`. The AXI4 master goes through an AXI clock converter and
interconnect to the MIG DDR3 SODIMM controller.

## Known limits and assumptions

- **Payload byte order** of the 7-series AXI-Stream interface is assumed little-endian per DW.
  If register writes or data come out byte-swapped on hardware, set `PAYLOAD_BSWAP = 1`.
- No SGLs, no Write Zeroes / Dataset Management, no Compare; ONCS = 0.
- Get Log Page ignores the log page offset.
- Only physically contiguous queues (CAP.CQR = 1).
- A controller reset finishes the command in progress before CSTS.RDY clears.
- One command at a time: performance is not tuned yet. A compress blocks other I/O while it runs.
- Compression uses fixed Huffman codes only: text compresses to about 29% (zlib level 6: 12%).
  On the board the test's word-list text gives 33% (zlib level 6: 15%), and 128 KB of zeros
  gives 1280 bytes (zlib: 149). A dynamic-Huffman encoder is the next ratio step.
- Compress-to-Staging (81h) and Decompress-Read (86h) use MPTR. The Linux `nvme` driver refuses a
  passthrough metadata buffer on a namespace without metadata (EINVAL), so 81h can only be sent
  from a user-space driver (VFIO, SPDK) or a kernel module; 86h works through Linux with NOCHK.
- With an IOMMU in DMA / DMA-FQ mode, Linux caps NVMe transfers at 128 KB (`max_hw_sectors_kb`),
  below MDTS. FFE-Send of a 128 KB record needs 128 KB + 256 B and is refused, so through Linux
  FFE records top out at 124 KB. With stock Linux there is no complete compress path for
  128 KB records. See [docs/compression_spec.md](docs/compression_spec.md#what-works-through-the-linux-nvme-driver).
- Two correctable Receiver Errors (physical layer) are logged at the upstream root port when the
  link retrains after a function reset, for example when the board moves between `vfio-pci` and
  `nvme`. None were seen during data transfers.
- The internal datapath moves 8 bytes per cycle: about 1 GB/s each way at 125 MHz, below the
  ~1.6 GB/s the link can carry. Widening the DMA paths to 4 DWs would close the gap.
- Engines run at about 65 MB/s (deflate) and 70 MB/s (inflate) at 125 MHz, below the
  200-250 MB/s target; match finding and Huffman decoding are bit- / byte-serial.
- Staging slots are freed by Commit, FFE-Receive, Commit with DISCARD or a controller reset; there
  is no timeout yet.
- FFE subset: at most 8 descriptors in the first page of PRP1, one source and one function,
  object-level CU, no PI, no partial object, compaction or filter; parameter data a 128-byte multiple.
