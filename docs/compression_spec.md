# Compression path: command interface (draft 1)

Implements the streaming two-phase write and single-phase read from
*ZC706 NVMe RAM Disk: Streaming Two-Phase Write and Read Path* (Nov 13, 2026), plus a
subset of the draft SNIA FFE-Send / FFE-Receive commands (*Fixed Function Data Structure
Model*, *FFE diagrams*) for compress (82h) and decompress (83h).

* Compression format: zlib stream (RFC 1950) around a raw deflate stream (RFC 1951).
  Stage 1 is LZ77 match finding (32 KB window), stage 2 is Huffman coding.
  The encoder emits fixed-Huffman blocks; the decoder accepts stored, fixed and dynamic
  blocks, so it reads any zlib stream (for example one written by ZFS's software gzip).
* Checksum: ZFS Fletcher-4 (four 64-bit sums over 32-bit little-endian words) of the
  stored bytes, zero padded to a 4 KB multiple (ashift = 12). Adler-32 of the uncompressed
  data is also reported.
* Standard Read / Write never compress: the namespace is **RAW by default**.

## Build parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `REC_LOG2` | 17 | Largest record: 17 = 128 KB (required) ... 20 = 1 MB (optional build) |
| `MDTS` | `REC_LOG2 - 11` | Max transfer = 2 x largest record (FFE parameter data = descriptors + record) |
| `RING_MB` | 64 | Staging ring size in DDR |
| Slot stride | 4 x record | raw area (2 x record) + output area (2 x record) |
| Slots | `RING_MB` / stride | 128 slots at 128 KB, 16 slots at 1 MB |
| `NSZE` | (1 GiB - 64 MiB) / 4 KiB = 245,760 | Namespace size; the ring sits right after it |

DDR map: `DDR_BASE + LBA x 4 KB` for the namespace, then the ring at
`RING_BASE = DDR_BASE + NSZE x 4 KB`. Slot *s* raw area at `RING_BASE + s x stride`,
output area at `+ stride / 2`.

## Set / Get Features, FID C0h: Compression Configuration

| CDW11 bits | Field | Notes |
| --- | --- | --- |
| 0 | CEN | 0 = RAW (default after reset / CC.EN), 1 = compression commands enabled |
| 7:4 | RECSZ | Record size limit = 128 KB << RECSZ (0 ... 3); above the build limit -> Invalid Field |
| 15:12 | (Get) MAXRS | Largest RECSZ this build supports |
| 31:16 | (Get) NSLOT | Number of staging slots |

Compress-to-Staging, Commit and FFE-Send return Command Sequence Error (00h/0Ch) when
CEN = 0. Decompress-Read and FFE-Receive work in either mode.

## Vendor I/O commands (NVM command set, NSID = 1)

Opcode bits 1:0 follow the NVMe data-direction convention.

### 81h Compress-to-Staging (phase 1, host -> device)

| Field | Meaning |
| --- | --- |
| PRP1/PRP2 | Uncompressed record, LSIZE bytes |
| CDW10 | LSIZE in bytes: 512-byte multiple, 1 ... record size limit |
| CDW11 | 7:0 threshold percent T (0 = ZFS rule: stored size must be <= LSIZE - LSIZE/8) |
| MPTR | 64-byte Result Record written by the device (DW aligned) |
| CQE DW0 | 15:0 handle, 16 INCOMP |

The device copies the record into a free slot, deflates it into the slot's output area,
zero pads to 4 KB and computes Fletcher-4. If the zlib stream is larger than the
threshold allows, the record is marked incompressible and the raw data (padded) is kept.

### 84h Commit (phase 2, no data)

| Field | Meaning |
| --- | --- |
| CDW10-11 | SLBA |
| CDW12 | 15:0 NLB, 0's based; must equal PSIZE / 4 KB - 1 |
| CDW13 | 15:0 handle; bit 31 DISCARD: free the slot without writing |

Copies PSIZE stored bytes of the slot to `SLBA` and frees the slot.

### 86h Decompress-Read (device -> host)

| Field | Meaning |
| --- | --- |
| CDW10-11 | SLBA of the compressed extent |
| CDW12 | 15:0 NLB, 0's based (extent = PSIZE = (NLB + 1) x 4 KB) |
| CDW13 | CSIZE: zlib stream length in bytes, 1 ... PSIZE |
| CDW14 | ULEN: uncompressed length in bytes, multiple of 4, <= record size limit |
| CDW15 | bit 0 NOCHK: skip the Fletcher-4 check |
| PRP1/PRP2 | Output buffer, ULEN bytes |
| MPTR | 64-byte descriptor; the device reads the expected Fletcher-4 at offset 10h (a Result Record can be passed back unchanged) |

Exactly ULEN bytes are always transferred (zero filled after an error).

### Result Record / CRB (64 bytes, little endian)

| Offset | Size | Field |
| --- | --- | --- |
| 00h | 2 | Handle |
| 02h | 2 | Flags: bit 0 INCOMP (raw data stored), bit 1 VALID, bit 2 DECOMP (record is a decompress result) |
| 04h | 4 | LSIZE: uncompressed bytes |
| 08h | 4 | CSIZE: zlib stream bytes (= LSIZE when INCOMP) |
| 0Ch | 4 | PSIZE: stored bytes (CSIZE rounded up to 4 KB) |
| 10h | 32 | Fletcher-4 of the PSIZE stored bytes: A, B, C, D, each 64 bits |
| 30h | 4 | Adler-32 of the uncompressed data |
| 34h | 2 | List ID (FFE only) |
| 36h | 2 | Status (as in the CQE: SCT << 8 | SC) |
| 38h | 8 | Reserved |

### Command-specific status (SCT = 1)

| SC | Meaning |
| --- | --- |
| C0h | Staging Full: no free slot, retry later |
| C1h | Invalid Handle / unknown List ID |
| C2h | Checksum Mismatch |
| C3h | Decompression Error (bad zlib header, bad block, bad distance, Adler-32 mismatch) |
| C4h | Length Mismatch (stream ended before ULEN or produced more) |

## FFE subset

### 89h FFE-Send (host -> device), 8Ah FFE-Receive (device -> host)

| Field | FFE-Send | FFE-Receive |
| --- | --- | --- |
| CDW2 15:0 | List ID | List ID |
| CDW10 | Parameter data length (bytes) | Buffer length (bytes) |
| CDW11 | 15:0 segment descriptors, 31:16 function descriptors (FPD counted) | - |
| CDW12 | Inline data length | - |
| PRP1/PRP2 | Parameter data | CRB (64 B) then output data |
| CQE DW0 | 15:0 handle, 16 INCOMP | Bytes transferred |

Parameter data = 64-byte descriptors, then inline data. Restrictions of this
implementation: at most 8 descriptors (512 bytes), all in the first page of PRP1; one
source; one function (82h or 83h) plus its FPD (FEh); CU size 0 (object level); no PI,
no partial object, no compaction, no filter.

FFE-Receive returns the CRB, then the retained output (decompressed data for 83h,
stored bytes for an 82h with no LBA destination), and frees the List ID's slot.

### Descriptors used

| Type | Bytes used |
| --- | --- |
| 01h Destination (LBA form) | 1 bit 0 DLBA: write the result to SLBA (else retain for FFE-Receive); 12-19 SLBA; 20-21 NLB (0's based) |
| 03h LBA Source | 4-7 NSID (0 or 1); 8-15 SLBA; 16-17 NLB (0's based) |
| 04h Inline Source | 4-7 length; 8-11 offset in parameter data (DW aligned) |
| 82h Compress FD | 6-7 number of sources (1); 16-17 source descriptor index |
| 83h Decompress FD | 6-7 number of sources (1); 16-17 source descriptor index |
| FEh FPD (82h) | 5 algorithm (01h = zlib); 6 level (ignored); 7 threshold percent; 8-11 CU size (0); 12-15 alignment (0 or 4096) |
| FEh FPD (83h) | 4 algorithm (01h = zlib); 8-11 / 12-15 filter offset / length (0) |

82h takes an inline source (the record). 83h takes an inline or LBA source (the zlib
stream); with an LBA source its length is NLB blocks and the stream end marks CSIZE.

## Using it from a host

1. Enable compression: Set Features FID C0h, CDW11 = 1 (CEN = 1, RECSZ = 0 for 128 KB
   records). Get Features C0h returns NSLOT in bits 31:16 (128 on the 128 KB build). Standard
   Read / Write keep working in both modes; the namespace stays RAW until CEN is set.
2. Two-phase write of one record:
   * **81h Compress-to-Staging** with the record in PRP1/PRP2, CDW10 = LSIZE, CDW11 = threshold
     (0 = ZFS rule) and MPTR pointing at a 64-byte host buffer. CQE DW0 has the handle and INCOMP;
     the Result Record at MPTR has CSIZE, PSIZE, Fletcher-4 and Adler-32.
   * Allocate PSIZE / 4 KB blocks, then **84h Commit** with that SLBA, NLB = PSIZE / 4 KB - 1 and
     the handle. Several records can be staged before any is committed (up to NSLOT), and they
     can be committed in any order. Commit with DISCARD (CDW13 bit 31) drops a record. A Commit
     with the wrong NLB is refused with Invalid Field and the record stays staged.
3. Read a compressed record back with **86h Decompress-Read**: SLBA, NLB, CSIZE and ULEN from the
   Result Record, and MPTR pointing at that same record (the device takes the expected
   Fletcher-4 from offset 10h), or CDW15 bit 0 (NOCHK) to skip the check. An incompressible
   record (INCOMP) is stored raw: read it with a plain Read.
4. Set CEN = 0 again to go back to RAW. A controller reset (CC.EN 1 -> 0) also returns to RAW and
   frees every slot.

### What works through the Linux nvme driver

The vendor commands can be sent with the standard passthrough ioctl (`NVME_IOCTL_IO_CMD`, as
`nvme io-passthru` does), with two host-side limits, both measured on Linux 7.0 with the IOMMU
in DMA-FQ mode (`scripts/vs_passthru_check.py`, 2026-11-26):

* **No metadata buffer.** A passthrough command with a metadata buffer on a namespace without
  metadata fails with EINVAL in the kernel, so MPTR cannot be supplied. Without one the kernel
  leaves MPTR at 0.
* **128 KB per transfer.** nvme-pci caps `max_hw_sectors_kb` at the IOMMU's optimal mapping size,
  128 KB, below the board's MDTS (256 KB). The other NVMe drive in the host has the same cap.
  Larger commands fail with EINVAL before they reach the board.

| Command | Through Linux | Notes |
| --- | --- | --- |
| 81h Compress-to-Staging | No | Needs MPTR for the Result Record |
| 84h Commit | Yes | No data; vendor status codes (00Ch, 1C1h, ...) come back in the ioctl result |
| 86h Decompress-Read | Only with NOCHK = 1 | Verified up to a full 128 KB record (ULEN = 128 KB fits). With NOCHK = 0 the device would read the checksum from address 0: do not send |
| 89h FFE-Send 82h | Records up to 124 KB | 256 B of descriptors + the record must fit 128 KB |
| 89h FFE-Send 83h | Yes | Inline stream or LBA source |
| 8Ah FFE-Receive | Up to 128 KB - 64 B of output | The 64 B CRB comes first |

So with stock Linux and an IOMMU, a 128 KB ZFS record has no complete compress path: 81h needs
MPTR and FFE-Send needs 256 B more than the host allows. Decompress of a 128 KB record works
with 86h and NOCHK (FFE-Receive cannot return it: CRB + 128 KB is over the limit). For full
128 KB records drive the controller from user space through VFIO, as
`scripts/staging_hwtest.py` does.

## Testing on hardware

| Script | Covers | Path to the board |
| --- | --- | --- |
| `scripts/ffe_hwtest.py` | FFE-Send / FFE-Receive: 82h compress, 83h decompress (inline and LBA source / destination), software zlib streams at levels 0, 1, 6, 9, error codes, rates | Linux `nvme` driver, passthrough ioctls on `/dev/nvme1` |
| `scripts/staging_hwtest.py` | 81h, 84h, 86h: the two-phase write and Decompress-Read | `vfio-pci`, polled user-space NVMe driver |
| `scripts/vs_passthru_check.py` | Which vendor commands the Linux driver passes: 84h status codes, 81h with a metadata buffer, 86h with NOCHK (FFE-written extents, software zlib 128 KB records, error codes) | Linux `nvme` driver, passthrough ioctls |

`ffe_hwtest.py` and `vs_passthru_check.py` read `max_hw_sectors_kb` and shrink their largest
record to fit (124 KB on a 128 KB host). Results on 2026-11-26: `ffe_hwtest.py` 245 of 245
checks, `vs_passthru_check.py` 20 of 20.

```bash
sudo python3 scripts/staging_hwtest.py              # finds the 10ee board; or pass 0000:08:00.0
sudo python3 scripts/staging_hwtest.py --keep-vfio  # leave it on vfio-pci afterwards
```

`staging_hwtest.py` moves the board to `vfio-pci`, which needs the board alone in its IOMMU
group and IOMMU enabled (Intel VT-d / AMD-Vi). The script then sets up the admin queue and one
I/O queue pair (64 entries, no interrupts) and maps 4 MiB of DMA memory at IOVA 0x1000_0000.
At the end it disables compression, clears CC.EN and rebinds the board to `nvme`.
It refuses to run if a namespace of the board is mounted, and writes only LBAs >= 200000. It
checks:

* 81h and 84h refused with Command Sequence Error while RAW
* For zeros, text, runs and random records of 512 B, 4 KB, 13.5 KB, 64 KB and 128 KB:
  * the CQE handle and INCOMP, and the Result Record: VALID, LSIZE, PSIZE = CSIZE rounded up to
    4 KB, Adler-32 and the ZFS 12.5% rule
  * after Commit, a plain Read of the extent: Fletcher-4 matches the record, Python's
    `zlib.decompress` of the stored stream gives back the record, the padding is zero (or the
    raw data for INCOMP), and the block after the extent is untouched
  * 86h Decompress-Read of the extent returns the record
* Threshold percent on both sides of the boundary `CSIZE x 100 <= LSIZE x T`
* 8 records staged at once (distinct handles), committed in reverse order, all verified
* Errors:
  * bad LSIZE (0, not a 512 multiple, above the record size)
  * NLB too large, then the right NLB succeeds
  * freed handle, handle >= NSLOT, LBA range
  * DISCARD, then the handle is gone
  * Staging Full after exactly NSLOT staged records, and the slots free again after discards
  * wrong Fletcher-4 (C2h), NOCHK, ULEN short (C4h)
* 128 KB stage + commit and Decompress-Read rates

Results on 2026-11-26 (ZC706, 128 KB build): **349 of 349 checks passed.**

| Record | 512 B | 4 KB | 13.5 KB | 64 KB | 128 KB | zlib level 6, 128 KB |
| --- | --- | --- | --- | --- | --- | --- |
| zeros, CSIZE | 13 | 48 | 143 | 645 | 1280 | 149 |
| text, CSIZE | 221 | 1409 | 4630 | 21924 | 43679 (3.00x) | 19539 |
| runs, CSIZE | 25 | 97 | 366 | 1767 | 3783 | 2905 |
| random | INCOMP | INCOMP | INCOMP | INCOMP | INCOMP | (larger than input) |

Stage + commit of 128 KB text ran at 50.5 MB/s (2.6 ms per record) and Decompress-Read at
31.1 MB/s (4.2 ms per record). These are one command at a time, polled from Python, so they
are lower bounds, not the engines' throughput.
