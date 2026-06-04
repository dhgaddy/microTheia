# Limitations, Results, and Future Work

## Current Results

### RTL Simulation

All individual module testbenches pass under RTL simulation with Icarus Verilog. The tested modules are:

- `evt2_decoder` — correct coordinate mapping, TIME_HIGH reconstruction, weight/threshold routing.
- `input_fifo` — correct backpressure and bypass behavior.
- `sram_wrapper` — correct read/write protocol for single- and multi-bank configurations.
- `voxel_binning` — correct RMW accumulation, bin rollover, and feature window readout.
- `voxel_mac_engine` — correct signed dot-product results.
- `voxel_gesture_classifier` — correct argmax and threshold decisions.
- `control_fsm` — correct state transitions through BOOT → LOAD → RUN.
- `spi_wrapper` — correct 32-bit framing and MISO output encoding.

Integration-level (`voxel_bin_core`, `soc`) and chip-level (`chip_top`) simulations exercise the full pipeline with pre-recorded EVT2 gesture data.

### Gate-Level Simulation

Gate-level simulation on the post-PnR netlist passes functional checks for all four gesture classes. Each GL simulation run takes approximately **7 hours** on a modern workstation.

### Implementation

The design targets the **GF180MCU** process via the wafer.space MPW shuttle using the LibreLane EDA flow. Implementation through place-and-route completes without errors using the `leo/gf180mcu` branch of LibreLane.

---

## Known Limitations

### Fixed Number of Classes

`NUM_CLASSES` is fixed at 4 throughout the design. The gesture classifier pipeline is hardcoded with 4-input pair-wise argmax stages. Adding a fifth class would require redesigning the classifier and adding a fifth weight SRAM, with corresponding changes to the SPI output encoding. The current 2-bit `gesture` output inherently cannot represent more than 4 classes.

### No Polarity Distinction

EVT2 CD events carry an on/off polarity bit (type `0x0` vs `0x1`). The current decoder **ignores polarity** — both polarities increment the same counter at the same (x, y) location. Incorporating polarity would double the feature space (requiring twice the counter memory and MAC compute) and would likely improve motion direction discrimination.

### Integer-Only Inference

All weights are 8-bit signed integers and all event counts are 16-bit unsigned integers. There is no floating-point support and no normalization layer. The model must be quantized to int8 before loading. This limits the representable function complexity and may degrade accuracy for tasks that benefit from finer-grained weight resolution.

### Saturating Counters

Event counters saturate at `2^COUNTER_BITS − 1` (65535 with 16-bit counters). For very high event rates or very long bins, all active cells will be saturated and relative event density information is lost. This primarily affects scenes with high scene motion or bright ambient light causing many spurious events.

### Single-Port SRAM Constraint

All SRAMs are single-port (read or write in a given cycle, not both). The most impactful consequence is in `voxel_binning`: during a RMW cycle, the accumulator must stall the incoming event for 2 cycles. This halves the peak event throughput to 32 Mevents/s. A true-dual-port SRAM macro would allow pipelined single-cycle accumulation.

### SRAM Hazard in Synthesis

A simultaneous read and write to *different* addresses within the same single-port SRAM cycle causes the read to be silently dropped in synthesis (single-port macro constraint). The RTL is written to avoid this in the normal event flow, but there is no hardware interlock — incorrect usage by future modifications would be silent.

### No Error Detection or Correction

There is no parity, CRC, or other error detection on the SPI stream or in the SRAM array. Bit-flips caused by electrical noise, radiation, or marginal timing will produce silently incorrect behavior. This is acceptable for an experimental academic chip but would need addressing for safety-critical applications.

### SPI-Only Host Interface

The only way to communicate with the chip (send events, load weights, read classifications) is via the SPI slave interface. There is no UART, I2C, or other secondary interface. If the SPI host (typically the DVS camera or a microcontroller) fails, the chip cannot be reconfigured without a power cycle followed by a new BOOT_REQ + weight load sequence.

### Temporal Binning is Irreversible

Once `EVT_READS_DONE` is received and the FSM transitions to RUN, events accumulate continuously. There is no pause, flush, or synchronization primitive. If the host needs to align the beginning of a gesture window to a known time boundary, it must either use a `RELOAD_REQ` (full weight reload) or accept an arbitrary phase offset in the first window.

### No Temporal Interpolation

Bin boundaries are discrete: an event either falls in bin N or bin N+1. There is no soft temporal weighting (e.g., bilinear interpolation across bins). This causes discontinuities at bin boundaries that may degrade accuracy for motions occurring exactly at a boundary.

### Sensor Resolution Hardcoded to 320×320

The `SENSOR_WIDTH` and `SENSOR_HEIGHT` parameters default to 320×320. Other Prophesee sensor resolutions (e.g., 640×480 Metavision, 1280×720) would require different compression ratios and recompilation. The 16×16 grid keeps spatial resolution very coarse regardless.

### Waveform File Size

Gate-level FST waveform files grow to tens of gigabytes for the full 7-hour GL simulation. Disk space of at least 50 GB free is recommended before running `make sim-gl-parallel`.

---

## Things That Don't Work

### SDF Back-Annotation (`make sim-sdf`)

SDF back-annotation for timing-accurate gate-level simulation is not fully functional. The GF180MCU SRAM Verilog timing models required for SDF annotation are not consistently available in the open PDK distribution. When models are missing, `iverilog` drops the timing annotations silently and the simulation runs as a functional (zero-delay) simulation.

### iCE40 FPGA Flow

The iCE40 FPGA target (`make ice40`) serves as a faster prototype iteration loop, but the full 16×16 grid configuration does not fit in the resource-constrained iCE40. The reduced `voxel_sim_fast` configuration (8×8 grid, 8 bins) is used for FPGA prototyping. See [`ice40/README.md`](../ice40/README.md) for board-specific instructions.

### Gate-Level Simulation Parallelism

`make sim-gl-parallel` is supposed to run four gesture tests in parallel (one per class). In practice, Icarus Verilog and cocotb generate conflicting temporary filenames in `cocotb/sim_build/` when run from the same directory, requiring careful workspace isolation. The workaround is to run each gesture test sequentially or in separate build directories.

### Page 3 of Debug Mux

Debug page 3 (`debug_select = 4'b0011`) is listed as "reserved for control module debug signals" but is currently tied to all zeros. `control_fsm` exports `main_state_dbg_o` and `load_state_dbg_o` signals that were intended to be wired here but the connection was not completed.

---

## Shortcomings and Things to Improve

### Training Infrastructure

There is no integrated training pipeline. Weights must be computed externally (e.g., using scikit-learn or a custom NumPy script on EVT2 recordings), manually quantized to int8, and loaded via the EVT2_WEIGHT command sequence. A proper training loop that directly produces quantized weights compatible with the chip format would significantly lower the barrier to trying new gesture sets.

### Threshold Tuning is Manual

Classification thresholds must be determined experimentally by observing raw scores on the debug bus and adjusting until the desired precision/recall trade-off is achieved. There is no automated calibration procedure.

### No Streaming Output

The chip emits one classification per feature window. If the host needs a confidence stream at a higher temporal resolution, there is no sub-window output mechanism. Adding a mode where partial-window scores are emitted periodically would help latency-sensitive applications.

### Memory Initialization

After a power cycle, all SRAMs contain unknown values. The chip will not operate correctly until a full weight/threshold load sequence completes (`BOOT_REQ` → `EVT_READS_DONE`). If power is lost during loading, the chip must restart the load from the beginning. A built-in non-volatile default weight set or a checksum-based load verification would improve robustness.

### Debug Bus Bandwidth

Only 32 bits of debug state are visible at a time. Capturing a multi-cycle event or tracing a sequence of operations requires sending multiple `DEBUG_PAGE` commands and reading back results over multiple SPI transactions. A FIFO-backed capture buffer would enable post-hoc trace capture.

### Scalability of MAC Engine

The MAC engine streams through features sequentially. Adding more classes or larger grids increases latency linearly. For a 32×32 grid (1024 cells × 16 bins = 16384 features) at 4 classes, each inference takes 16386 cycles ≈ 256 µs. For real-time applications at 30 Hz this is fine, but for burst-mode inference at higher rates it becomes a bottleneck. A parallel-class MAC unit (processing all 4 classes in one pass per feature address) is already partially implemented (reading from all 4 weight SRAMs simultaneously) but the accumulator stages could be further pipelined.

### Cocotb 2.0 Migration

The testbenches target cocotb 2.0.1. The cocotb 2.0 API changed coroutine syntax relative to 1.x (no `@cocotb.coroutine` decorator; plain `async def`). Any testbench code written against cocotb 1.x will fail without migration. This is noted because some online examples and LLM-generated cocotb code still uses the older API.
