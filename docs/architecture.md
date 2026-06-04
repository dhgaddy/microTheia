# μTheia Architecture

## Overview

μTheia is a fixed-function neuromorphic inference ASIC fabricated on the GF180MCU process.
It accepts a stream of [EVT2](https://docs.prophesee.ai/stable/data/encoding_formats/evt2.html)-encoded events from a dynamic vision sensor (DVS) camera over SPI, builds a spatio-temporal feature volume (a *voxel grid*), computes integer dot-product scores against four sets of learned weights, and emits a gesture classification result back over SPI.

The chip is designed around a single principle: **all computation is streaming and in-order**. There is no CPU, no cache, and no general-purpose memory. Every stage is a purpose-built datapath connected by simple valid/ready handshakes.

---

## System Block Diagram

```
DVS Camera (SPI master)
        │
        │ SPI (MOSI 32-bit words, MISO classification)
        ▼
┌─────────────────────────────────────────────────────────┐
│  chip_top  (IO pads, power rings, wafer.space IPs)      │
│                                                         │
│  ┌──────────────────────────────────────────────────┐   │
│  │  chip_core  (pin mux, heartbeat, MISO routing)   │   │
│  │                                                  │   │
│  │  ┌───────────────────────────────────────────┐   │   │
│  │  │  soc                                      │   │   │
│  │  │                                           │   │   │
│  │  │  ┌──────────────┐   ┌─────────────────┐   │   │   │
│  │  │  │ spi_wrapper  │──▶│ voxel_bin_core  │   │   │   │
│  │  │  │              │◀──│                 │   │   │   │
│  │  │  └──────────────┘   └─────────────────┘   │   │   │
│  │  └───────────────────────────────────────────┘   │   │
│  └──────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────┘
```

```
voxel_bin_core internal pipeline
─────────────────────────────────────────────────────────────────
SPI word  ->  input_fifo  ->  evt2_decoder  ->  voxel_binning
                                                     │
                                              feature_ram  (SRAM)
                                                     │
                                          voxel_mac_engine  <-  weight_sram[0..3]
                                                     │             thresh_sram
                                       voxel_gesture_classifier
                                                     │
                                           gesture / gesture_valid
```

---

## Pipeline Stages

### 1. IO & Clock Distribution (`chip_top` / `chip_core`)

`chip_top` instantiates the GF180MCU IO pad ring (signal, power, ground pads) and the wafer.space identification IPs. All pad signals are wired directly to `chip_core`.

`chip_core` handles:
- **Pin multiplexing.** A rising edge on `ALT_INPUT_MODE` (pin 8) switches SPI signals between two physical pin sets (pins 2–4 vs pins 5–7). This allows the board layout to use whichever physical location is more convenient.
- **MISO routing.** The MISO output pad is selected based on ALT_INPUT_MODE, with tri-state control from the SPI slave's chip-select line.
- **Heartbeat.** `bidir[0]` toggles at 0.5 Hz (1 second on, 1 second off at 64 MHz) as a liveness indicator.
- **Debug bus.** `bidir[6:37]` carry a 32-bit selectable debug bus from `selectable_debug` inside the core.

### 2. SPI Slave Interface (`spi_wrapper` + `spi_module`)

The SPI slave (third-party `spi_module.v`, Jan Schiefer) shifts in 32-bit words MSB-first. `spi_wrapper` adds:
- A **word-holding register** — data is forwarded downstream only on complete 32-bit transfers. If CS de-asserts mid-word the partial word is discarded.
- **Output mux** — the upper 3 bits of the MISO shift register carry `{gesture_confidence, gesture[1:0]}` from the most recent classification.

The chip is purely SPI slave; the DVS camera or host MCU is always master.

### 3. Input Buffering (`input_fifo`)

A 256-deep, 32-bit SRAM-backed FIFO decouples the SPI data rate from the event decoder. The FIFO uses a *tail-only* design with direct bypass when empty: new data is forwarded to the output register in the same cycle if the FIFO is empty, avoiding unnecessary latency.

Backpressure propagates via the standard ready/valid protocol. If the FIFO fills (256 full words) the SPI wrapper holds further data until space is available.

### 4. EVT2 Decoding (`evt2_decoder`)

Each 32-bit word is classified by its upper 4 bits:

| Type [31:28] | Meaning |
|---|---|
| `0x0`, `0x1` | CD event (change detection, on/off polarity) |
| `0x8` | Time High (extends timestamp to 34 bits) |
| `0x2` | Weight load command |
| `0x3`, `0x4` | Threshold upper/lower word |
| `0x5`, `0x6` | Bin length upper/lower word |
| `0x7` | Per-bin spatial boundary (VOXEL_DIMS) |
| `0xA–0xF` | Control commands (debug, reload, boot, etc.) |

For CD events the decoder:
1. Reconstructs a 34-bit timestamp (28-bit time-high + 6-bit event LSB).
2. Maps the raw 11-bit X/Y sensor coordinates (up to 2048) into 4-bit grid coordinates (0–15) using per-bin boundary tables stored in registers.
3. Emits `(grid_x, grid_y, timestamp, polarity_ignored)` to the binning stage.

Weight and threshold commands are routed directly to SRAM write ports and do not pass through the binning pipeline.

### 5. Temporal Binning (`voxel_binning`)

This is the core accumulation engine. It maintains a **ring buffer of NUM_BINS (16) temporal bins**, each holding a 16×16 grid of saturating event counters.

**Accumulation (ST_ACCUM):**
Every incoming event triggers a 2-cycle read-modify-write:
- Cycle N: Read the current counter at `(bin_idx, x, y)`.
- Cycle N+1: Write back `counter + 1` (saturating at `2^COUNTER_BITS − 1`).

This means event throughput is **one event per two clock cycles** — 32 Mevents/s at 64 MHz.

**Bin rollover:**
The decoder compares the event timestamp against a running `start_ts + bin_length`. When the timestamp crosses the boundary, the current bin index advances and accumulation continues into the next bin. The event that triggered the rollover is held and reprocessed in the new bin.

**Readout (ST_READOUT):**
When `completed_bins` reaches `READOUT_BINS` (16 by default), the oldest NUM_BINS bins are read out sequentially and written to the feature RAM. This forms one complete spatio-temporal feature window for classification.

**Clearing (ST_CLEAR):**
After readout the consumed bins are zeroed before reuse.

### 6. Feature RAM (Double-Buffered SRAM)

The feature window (`GRID_SIZE² × READOUT_BINS` entries of `COUNTER_BITS` bits) is stored in GF180MCU SRAM macros via `sram_wrapper`. The double-buffer scheme allows the MAC engine to read one window while the binning stage writes the next, preventing pipeline stalls.

### 7. MAC Scoring (`voxel_mac_engine`)

Once a feature window is ready, the MAC engine streams through all `FEATURE_COUNT = GRID_SIZE² × READOUT_BINS` addresses in order:

```
score[c] = Σ feature[i] × weight[c][i]   for i in 0..FEATURE_COUNT-1
```

- Features are `COUNTER_BITS`-wide unsigned integers.
- Weights are 8-bit **signed** integers (int8, range −128..127) stored in per-class SRAMs. Negative weights act as negative evidence (suppress the score for events at that location/time).
- The accumulator is `SCORE_BITS = COUNTER_BITS + WEIGHT_BITS + ceil(log2(FEATURE_COUNT)) + 1` bits wide to prevent overflow.

All four class scores are computed in a single sequential pass (the engine reads the same feature address from all four weight SRAMs simultaneously).

Computation latency: `FEATURE_COUNT + 2` cycles. At 16×16×16 features this is 4098 cycles ≈ 64 µs at 64 MHz.

### 8. Gesture Classification (`voxel_gesture_classifier`)

A 4-stage pipeline performs:
1. **Argmax** — find the class with the highest score.
2. **Threshold check** — compare `max_score` against a per-class threshold read from SRAM (addresses 0–3).
3. **Confidence check** — compare `(max_score − second_score)` against a per-class difference threshold (addresses 4–7).
4. **Output** — assert `gesture_valid` if the score threshold passes; assert `gesture_confidence` if the margin threshold also passes.

Both thresholds are signed comparisons, which allows negative bias (always suppress) or very permissive tuning.

### 9. Debug Infrastructure (`selectable_debug`)

All 32-bit debug signals are latched behind flops to break long combinational paths. An EVT2 `DEBUG_PAGE` command selects one of 11 pages at runtime; see [`debug_mux_pinout.txt`](../debug_mux_pinout.txt) for per-bit assignments.

---

## Memory Map

| SRAM | Depth | Width | Purpose |
|------|-------|-------|---------|
| counter_sram | NUM_BINS × GRID_SIZE² | COUNTER_BITS | Voxel event counters (binning) |
| feature_ram | GRID_SIZE² × READOUT_BINS | COUNTER_BITS | Feature window (double-buffered) |
| weight_sram[0..3] | FEATURE_COUNT | WEIGHT_BITS | Per-class weights |
| thresh_sram | 8 | SCORE_BITS | Class + diff thresholds |

All SRAMs are backed by tiled GF180MCU macros (256×8, 512×8, or 1024×8) selected automatically by `sram_wrapper` based on depth/width parameters.

---

## Control Flow (`control_fsm`)

```
BOOT ──(boot_req)──▶ LOAD ──(evt_reads_done)──▶ RUN ──(reload_req)──▶ BOOT
                      │                            │
                    (fail)                      (debug_req)
                      ▼                            ▼
                    BOOT                         DEBUG
```

- **BOOT:** Holds everything in reset until a `BOOT_REQ` EVT2 command arrives.
- **LOAD:** Gates SRAM write ports open so weight/threshold commands from the decoder flow into SRAMs. A 1024-cycle power-up delay (`LD_WAIT_PWR`) ensures SRAMs are stable before the first write.
- **RUN:** Normal inference mode. New boot/reload/debug requests are accepted.
- **DEBUG:** Core held in reset; debug bus is readable via SPI.

---

## Clocking and Reset

- Single clock domain: 64 MHz.
- Active-low asynchronous reset (`rst_n`) propagated from pad → `chip_core` → `soc` → all submodules.
- SPI clock is an asynchronous input; the SPI module handles its own synchronization. The `spi_ready` signal handshakes the boundary.
- There is no PLL. The 64 MHz clock is expected to come directly from the board.

---

## Design Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `CLK_FREQ_HZ` | 64,000,000 | Core clock |
| `GRID_SIZE` | 16 | Spatial grid dimension (N×N) |
| `NUM_BINS` | 16 | Ring buffer depth |
| `READOUT_BINS` | 16 | Feature window temporal depth |
| `COUNTER_BITS` | 16 | Saturating event counter width |
| `WEIGHT_BITS` | 8 | Signed weight width |
| `NUM_CLASSES` | 4 | Number of gesture classes |
| `SCORE_BITS` | 37 | Accumulator width (auto-derived) |
| `FIFO_DEPTH` | 256 | Input FIFO entries |
| `SENSOR_WIDTH` | 320 | DVS sensor X resolution |
| `SENSOR_HEIGHT` | 320 | DVS sensor Y resolution |

Reducing `GRID_SIZE` or `NUM_BINS` drastically reduces SRAM requirements and MAC latency but degrades classification accuracy because fewer spatial/temporal features are available.
