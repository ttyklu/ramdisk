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
