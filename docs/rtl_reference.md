# RTL Module Reference (`src/`)

All source files are SystemVerilog. The build order is defined in `rtl.f`.

---

## `chip_top.sv` — Top-Level with IO Pads

Instantiates the GF180MCU IO pad ring and the wafer.space identification IPs (QR code, shuttle ID, project marker, logo). The number and type of pads are driven by `slot_defines.svh` which is conditionally included based on the `SLOT` environment variable.

**Ports**

| Port | Direction | Width | Description |
|------|-----------|-------|-------------|
| `VDD`, `VSS`, `DVDD`, `DVSS` | supply | 1 | Power rails |
| `clk_PAD` | input | 1 | Core clock (64 MHz) |
| `rst_n_PAD` | input | 1 | Active-low reset |
| `input_PAD` | input | NUM_INPUT_PADS | Dedicated input signals |
| `bidir_PAD` | inout | NUM_BIDIR_PADS | Bidirectional signals |
| `analog_PAD` | inout | 2 | Unused analog pads |

**Notes**
- Pad library selected at build time by `PAD` env var (`gf180mcu_fd_io` or `gf180mcu_ocd_io`).
- The `generate` block switches pad cell names between library variants.

---

## `chip_core.sv` — Pin Mux and Top-Level Logic

Bridges pad signals to the internal SOC, implements pin multiplexing for the dual SPI interface, and generates the heartbeat signal.

**Key Internals**

| Signal | Description |
|--------|-------------|
| `ALT_INPUT_MODE` | Latch that toggles on rising edge of `input[7]` (pad pin 8). Routes SPI to pin set A or B. |
| `heartbeat` | Counter-driven 0.5 Hz toggle on `bidir[0]`. |
| `spi_ready` | Forwarded from `soc` to `bidir[1]`. |
| `debug_bus[31:0]` | Forwarded from `soc` to `bidir[6:37]`. |
| `MISO` | Routed to `bidir[38]` or `bidir[39]` depending on `ALT_INPUT_MODE`; unused MISO pad held via pull-down. |

**Pin Assignments (1×1 slot default)**

| Input Pad | Default Function | Alt Function |
|-----------|-----------------|--------------|
| 2 | SCLK (alt) | — |
| 3 | MOSI (alt) | — |
| 4 | CS (alt) | — |
| 5 | SCLK | — |
| 6 | MOSI | — |
| 7 | CS | — |
| 8 | ALT_INPUT_MODE | — |
| 9–11 | Pulled low | — |

---

## `soc.sv` — System-on-Chip Wrapper

Instantiates `spi_wrapper` and `voxel_bin_core`, connecting them via the EVT2 word stream interface.

**Parameters:** All parameters from both sub-modules are passed through. Refer to `voxel_bin_core` defaults.

**Connections**

```
spi_wrapper.evt_word      → voxel_bin_core.evt_word_i
spi_wrapper.evt_word_valid → voxel_bin_core.evt_word_valid_i
voxel_bin_core.evt_word_ready_o → spi_wrapper.evt_word_ready
voxel_bin_core.gesture_o        → spi_wrapper.gesture
voxel_bin_core.gesture_valid_o  → spi_wrapper.gesture_valid
voxel_bin_core.gesture_confidence_o → spi_wrapper.gesture_confidence
```

---

## `spi_wrapper.sv` — SPI Slave Wrapper

Wraps `spi_module.v` (third-party) with word-level framing and classification output.

**Ports**

| Port | Dir | Width | Description |
|------|-----|-------|-------------|
| `SCLK` | in | 1 | SPI clock (async) |
| `CS` | in | 1 | Chip select, active low |
| `MOSI` | in | 1 | Master out / slave in |
| `MISO` | out | 1 | Master in / slave out |
| `evt_word` | out | 32 | Received EVT2 word |
| `evt_word_valid` | out | 1 | Word valid pulse |
| `evt_word_ready` | in | 1 | Downstream ready (backpressure) |
| `gesture` | in | 2 | Class index from classifier |
| `gesture_valid` | in | 1 | Score above class threshold |
| `gesture_confidence` | in | 1 | Score margin above diff threshold |
| `spi_ready` | out | 1 | SPI slave ready indicator |

**Behavior**
- MISO upper 3 bits: `{gesture_confidence, gesture[1]}`, `gesture[0]` shifted through the 32-bit register.
- If CS rises before 32 bits are received, the incomplete word is dropped.
- `evt_word_valid` is a 1-cycle pulse aligned to the rising edge of CS after a complete word.

---

## `evt2_decoder.sv` — EVT2 Packet Decoder

Decodes 32-bit EVT2 words into typed events and control commands.

**Parameters**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `SENSOR_WIDTH` | 320 | DVS sensor X resolution |
| `SENSOR_HEIGHT` | 320 | DVS sensor Y resolution |
| `FEATURE_COUNT` | 4096 | GRID_SIZE² × READOUT_BINS |
| `GRID_SIZE` | 16 | Spatial grid N |
| `SCORE_BITS` | 37 | Score accumulator width |
| `WEIGHT_BITS` | 8 | Weight width |
| `REQUIRE_TIME_HIGH` | 0 | If 1, drop events without preceding TIME_HIGH |
| `SWAP_INPUT_BYTES` | 0 | Byte-swap each input word |

**Outputs — Event Stream**

| Signal | Width | Description |
|--------|-------|-------------|
| `x_out` | 4 | Grid X coordinate (0–15) |
| `y_out` | 4 | Grid Y coordinate (0–15) |
| `ts_out` | 34 | Reconstructed timestamp |
| `event_valid` | 1 | Event ready |
| `data_ready` | 1 | Decoder ready for input |

**Outputs — Weight Write Interface**

| Signal | Width | Description |
|--------|-------|-------------|
| `weight_addr_o` | 12 | Feature address |
| `weight_data_o` | 8 | Signed weight value |
| `weight_sram_addr_o` | 2 | Class index (0–3) |
| `weight_event_valid` | 1 | Write enable |

**Outputs — Control**

| Signal | Description |
|--------|-------------|
| `thresh_data_o`, `thresh_addr_o`, `thresh_event_valid` | Threshold SRAM write |
| `bin_length_us`, `bin_length_valid` | Bin duration update |
| `boot_req_o`, `reload_req_o`, `debug_req_o` | FSM control requests |
| `evt_reads_done` | End-of-load marker |
| `debug_page_sel` | Debug mux page select |

**EVT2 Packet Types**

| Type [31:28] | Command | Key Fields |
|---|---|---|
| `0x0`, `0x1` | CD event (off/on) | [27:22] ts_lsb, [21:11] x, [10:0] y |
| `0x8` | TIME_HIGH | [27:0] time_high |
| `0x2` | EVT_WEIGHT | [27:20] weight, [19:8] addr, [7:2] class_idx |
| `0x3` | THRESH_UPPER | [27:0] threshold[55:28] |
| `0x4` | THRESH_LOWER | [27:0] threshold[27:0] |
| `0x5` | BIN_LENGTH_UPPER | [27:0] bin_us[55:28] |
| `0x6` | BIN_LENGTH_LOWER | [27:0] bin_us[27:0] |
| `0x7` | VOXEL_DIMS | [27:24] bin_idx, [23:13] xbound, [12:2] ybound |
| `0xA` | DEBUG_REQ | — |
| `0xB` | RELOAD_REQ | — |
| `0xC` | BOOT_REQ | — |
| `0xE` | DEBUG_PAGE | [27:24] page_sel |
| `0xF` | EVT_READS_DONE | — |

---

## `input_fifo.sv` — Input FIFO

32-bit, `FIFO_DEPTH`-entry (default 256) SRAM-backed FIFO with single read/write ports.

**Architecture Notes**
- Tail-only pointer design: head is implicit (oldest entry always at head).
- If the FIFO is empty when data arrives, the data bypasses SRAM and goes directly to the output register (zero latency).
- Simultaneous read and write to the same SRAM bank causes the read to be deferred by one cycle.
- `ready_o` goes low when the FIFO plus the output register are both full (FIFO_DEPTH + 1 entries outstanding).

---

## `sram_wrapper.sv` — GF180MCU SRAM Abstraction

Provides a unified synchronous read/write interface over tiled GF180MCU SRAM macros.

**Parameters**

| Parameter | Description |
|-----------|-------------|
| `width_p` | Data bus width in bits |
| `depth_p` | Number of entries |

**Macro Selection**
The wrapper automatically selects and tiles GF180MCU macro variants:

| Macro | Depth | Width |
|-------|-------|-------|
| `gf180mcu_fd_ip_sram__sram256x8m8wm1` | 256 | 8 |
| `gf180mcu_fd_ip_sram__sram512x8m8wm1` | 512 | 8 |
| `gf180mcu_fd_ip_sram__sram1024x8m8wm1` | 1024 | 8 |

Multiple byte-lane macros are tiled horizontally (for wider buses), and multiple banks are tiled vertically (for deeper memories).

**Protocol**
- Write: data latched on rising clock edge when `WEN=0`, `CEN=0`.
- Read: address registered on rising edge; data appears at `Q` the following cycle (1-cycle latency).
- `CEN` must be held HIGH during reset.

**Simulation vs Synthesis**
- In simulation (`COCOTB` define set) the wrapper uses a behavioral `reg` array. This avoids timing issues with `clk_dly` in the macro models.
- In synthesis, `generate` blocks instantiate the real macros.

**Hazard Warning**
Simultaneous read and write to different addresses in the same cycle drops the read in synthesis (single-port macro constraint). In simulation this prints a warning. The surrounding logic (especially `voxel_binning`) is written to avoid this condition.

---

## `control_fsm.sv` — Boot/Load State Machine

**States**

| State | Description |
|-------|-------------|
| `ST_BOOT` | Idle, waiting for boot/reload/debug request |
| `ST_LOAD` | Weight/threshold loading active; SRAM write ports open |
| `ST_RUN` | Normal operation |
| `ST_DEBUG` | Core held in reset for debug inspection |

**Load Substates**

| Substate | Description |
|----------|-------------|
| `LD_IDLE` | Entry point |
| `LD_WAIT_PWR` | 1024-cycle SRAM power-up delay |
| `LD_OPEN` | Enable SRAM write ports (`evt_ld_en = 1`) |
| `LD_WAIT` | Wait for `evt_reads_done` signal |
| `LD_DONE` | De-assert `core_rst_o` to start RUN mode |
| `LD_FAIL` | Timeout or error path (returns to BOOT) |

**Outputs**

| Signal | Description |
|--------|-------------|
| `evt_ld_en` | Gates weight/threshold writes to SRAM |
| `core_rst_o` | Holds core submodules in reset during LOAD/DEBUG |
| `boot_done_o` | Pulses on successful transition to RUN |
| `boot_fail_o` | Pulses on load failure |

---

## `voxel_bin_core.sv` — Complete Inference Pipeline

Top-level module integrating all inference stages. Instantiates:

- `control_fsm`
- `input_fifo`
- `evt2_decoder`
- `voxel_binning`
- Feature RAM (`sram_wrapper`)
- Weight SRAMs × NUM_CLASSES (`sram_wrapper`)
- Threshold SRAM (`sram_wrapper`)
- `voxel_mac_engine`
- `voxel_gesture_classifier`
- `selectable_debug`

**Parameters** — full set passed from `soc`:

| Parameter | Default |
|-----------|---------|
| `CLK_FREQ_HZ` | 64,000,000 |
| `WINDOW_MS` | 1000 |
| `GRID_SIZE` | 16 |
| `NUM_BINS` | 16 |
| `READOUT_BINS` | 16 |
| `COUNTER_BITS` | 16 |
| `FIFO_DEPTH` | 256 |
| `DATA_WIDTH` | 32 |
| `SENSOR_WIDTH` | 320 |
| `SENSOR_HEIGHT` | 320 |
| `WEIGHT_BITS` | 8 |
| `NUM_CLASSES` | 4 |
| `SCORE_BITS` | 37 |

---

## `voxel_binning.sv` — Event Accumulation + Temporal Binning

**FSM States**

| State | Description |
|-------|-------------|
| `ST_ACCUM` | Accept and accumulate events (2-cycle RMW) |
| `ST_WAIT_RD` | Wait for SRAM read result before readout |
| `ST_READOUT` | Stream READOUT_BINS × GRID_SIZE² entries to feature RAM |
| `ST_CLEAR` | Zero the consumed bin entries |

**Accumulation Timing**
Every event requires exactly 2 clock cycles (RMW), so maximum event throughput is `CLK_FREQ_HZ / 2 = 32 Mevents/s`. Overflow events are held in a 1-entry pending register while the RMW completes.

**Bin Rollover Logic**
When an incoming event timestamp `ts` satisfies `ts >= start_ts + bin_duration_ts`, the binning stage:
1. Increments `wr_bin_idx` (wraps modulo NUM_BINS).
2. Advances `start_ts` by `bin_duration_ts`.
3. Holds the triggering event; reissues it into the new bin after 1 cycle.
4. Increments `completed_bins`.

When `completed_bins == READOUT_BINS`, the full temporal window is ready for readout.

**Readout Interface**

| Signal | Width | Description |
|--------|-------|-------------|
| `readout_start` | 1 | Asserted one cycle before readout begins |
| `readout_valid` | 1 | Readout data valid |
| `readout_last` | 1 | Last readout word in window |
| `readout_index` | 11 | Current address in feature window |
| `readout_data` | COUNTER_BITS | Counter value at this address |

---

## `voxel_mac_engine.sv` — Multiply-Accumulate

**FSM States**

| State | Description |
|-------|-------------|
| `ST_IDLE` | Waiting for `start` signal |
| `ST_STREAM` | Issuing sequential reads; accumulating with 1-cycle latency |
| `ST_PUBLISH` | Assert `scores_valid`, return to IDLE |

**Arithmetic**
```
product = signed(feature[i]) × signed(weight[c][i])   // 25-bit signed result
score[c] += product                                    // SCORE_BITS accumulator
```

Features are zero-extended to signed before multiplication; weights are already signed int8.

**Outputs**

| Signal | Width | Description |
|--------|-------|-------------|
| `scores_flat` | NUM_CLASSES × SCORE_BITS | All class scores, flattened |
| `scores_valid` | 1 | 1-cycle pulse when scores are ready |
| `score_A/B/C/D` | 32 | Lower 32 bits of each score (debug) |

---

## `voxel_gesture_classifier.sv` — Classification

4-stage combinational/registered pipeline. All comparisons are signed.

| Stage | Operation |
|-------|-----------|
| 0 | Register inputs |
| 1 | Pair-wise max: (score[0] vs score[1]), (score[2] vs score[3]) |
| 2 | Final argmax; compute second-place score and margin; read class threshold from SRAM |
| 3 | Compare max score vs class_threshold; read diff threshold from SRAM |
| 4 | Compare margin vs diff_threshold; emit `gesture`, `gesture_valid`, `gesture_confidence` |

**Threshold SRAM Layout**

| Address | Content |
|---------|---------|
| 0–3 | Per-class minimum score threshold |
| 4–7 | Per-class minimum margin (max − second) threshold |

---

## `selectable_debug.sv` — Debug Mux

Routes internal signals to a 32-bit debug bus based on a 4-bit `debug_page_sel` input.

All outputs are registered behind flops to prevent long combinational chains from limiting timing closure.

**Page Map Summary**

| Page | Content |
|------|---------|
| 0 | Classifier and MAC engine handshake/status signals |
| 1 | Voxel binning readout bus |
| 2 | Decoder, FIFO, and core-level status |
| 3 | Reserved (FSM state — currently open) |
| 4 | Decoder X/Y event output |
| 5 | FIFO input word |
| 6 | FIFO output word |
| 7 | Class A score (lower 32 bits) |
| 8 | Class B score |
| 9 | Class C score |
| 10 | Class D score |

See [`debug_mux_pinout.txt`](../debug_mux_pinout.txt) for exact per-bit assignments within each page.
