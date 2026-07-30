# μTheia

μTheia is a GF180MCU event-based machine-vision ASIC for motion-pattern classification from EVT2 event streams. The chip receives EVT2 event data and configuration commands over SPI, decodes timestamped events, compresses 320×320 sensor coordinates into a 16×16 spatial grid, bins activity into 16 programmable-duration temporal bins, stores feature windows in SRAM, performs integer MAC scoring against programmable class weights, and reports detections for four programmable classes through SPI or selectable physical debug/output pins.

<img width="1418" height="1846" alt="image" src="https://github.com/user-attachments/assets/81efcb82-028c-47d3-9f7e-0b72abdba28b" />


## Project summary

- **Process / flow:** GF180MCU, wafer.space MPW, LibreLane/OpenROAD, KLayout, Magic, and Netgen.
- **Input stream:** EVT2 event-camera words and configuration commands over SPI.
- **Feature extraction:** 320×320 coordinates are spatially compressed to a 16×16 grid and temporally binned into 16 programmable-duration bins.
- **Storage:** SRAM-backed feature and weight storage.
- **Classifier:** Integer MAC scoring for four programmable motion-pattern classes.
- **Output / debug:** Classification results are available over SPI and through selectable debug/output pins.
- **Frequency:** 64 MHz on-chip and 32 MHz SPI

### Project status: Submitted for manufacture.

## Documentation

Full technical documentation is in the [`docs/`](docs/) directory:

| Document | Contents |
|----------|----------|
| [`docs/architecture.md`](docs/architecture.md) | System architecture, block diagram, pipeline stages, memory map, and control flow |
| [`docs/rtl_reference.md`](docs/rtl_reference.md) | Per-module RTL reference covering ports, parameters, behavior, and timing |
| [`docs/simulation.md`](docs/simulation.md) | cocotb testbenches, utility functions, simulation configs, and waveform viewing |
| [`docs/limitations.md`](docs/limitations.md) | Known limitations, incomplete items, implementation status, and future improvements |

The [`docs/debug_mux_pinout.txt`](docs/debug_mux_pinout.txt) file documents per-bit assignments for the debug bus pages.

## Timing closure status @ 64 MHz

<img width="1156" height="277" alt="Screenshot 2026-06-18 at 4 44 27 PM" src="https://github.com/user-attachments/assets/2b3a0a7b-6eb6-45b2-b011-485fa4a000b1" />

### Positive setup and hold slack in all tested corners.

Note: Max capacitance violations are mostly pads being checked against the 0.2 pF global limit. The remaining max capacitance violations are all buffers within the clock net, again being checked against the aggressive 0.2 pF global limit. In both cases, all maximum capacitance violations were manually investigated, and the offending cells were checked to be within the maximum capacitances listed in their Liberty files. The maximum slew violations in the slowest corner are all bidirectional (assigned output) pads whose violations were similarly checked against their Liberty file entry and determined to be well within their characterized range.

### Full pass LVS and DRC

## Important caveats

Read these before running the project for the first time.

- **Nix/OpenROAD first-run compile time.** The `flake.nix` environment compiles OpenROAD from source using the `leo/gf180mcu` branch on the first `nix-shell` invocation. This can take **30–90 minutes**, depending on CPU speed. The result is cached locally, so later shell launches are much faster.
- **PDK setup is required before simulation.** Run `make config-pdk` once before RTL or gate-level simulation. Simulation depends on GF180MCU SRAM behavioral models from the PDK.
- **Gate-level simulation is slow and disk-intensive.** `make sim-gl` can take several hours, and chip-top/gate-level waveform files can reach tens of gigabytes. Make sure the machine has enough RAM, CPU cores, and free disk space before running long simulations.
- **Weights must be loaded before inference.** After every power cycle, the chip starts in the `BOOT` state and will not classify events until the full weight and threshold load sequence completes: `BOOT_REQ` → weight commands → `EVT_READS_DONE`. If the SPI master loses sync during loading, restart the load from the beginning.
- **SDF back-annotation is not fully reliable.** `make sim-sdf` is provided, but timing-accurate simulation is not currently dependable because GF180MCU SRAM Verilog timing models are not consistently present in the open PDK distribution. Icarus may silently fall back to zero-delay functional simulation.

## Prerequisites

The preferred flow uses the project Nix shell, which provides the required LibreLane/OpenROAD environment.

Install Nix and follow the LibreLane Nix installation instructions. Then activate the shell from the repository root:

```bash
nix-shell
```

The remaining commands assume that this shell is active.


## Quick start

```bash
git clone git@github.com:dolphin-530/microTheia.git
cd microTheia
nix-shell
make config-pdk
make sim
make librelane
make copy-final
make sim-gl
```

## Simulation

μTheia uses [cocotb](https://www.cocotb.org/) with Icarus Verilog for RTL and gate-level simulation. Testbenches are located in the `cocotb/` directory.

Run the default chip-top sanity test:

```bash
make sim
```

Run a specific module-level test:

```bash
make sim DUT=module_name CONFIG=config_name
```

If `CONFIG` is omitted, the simulation uses the default compile arguments from the Makefile. Configuration files are stored in the `configs/` directory.

Run the full RTL simulation suite:

```bash
make sim-all
```

Run gate-level simulation from the copied final implementation:

```bash
make sim-gl
```

> [!NOTE]
> Gate-level simulation expects the latest completed implementation to be copied into the `final/` directory. After a successful LibreLane run, use `make copy-final` before running `make sim-gl`.

Simulation waveforms are generated under `cocotb/sim_build/`. To open the default waveform view, run:

```bash
make sim-view
```

## Implementation

Enter the Nix environment:

```bash
nix-shell
```

Run the physical implementation flow:

```bash
make librelane
```

After the flow completes, copy the latest successful run into `final/`:

```bash
make copy-final
```

`make copy-final` only works when the most recent implementation run completed successfully.

## Viewing the design

Open the completed design in the OpenROAD GUI:

```bash
make librelane-openroad
```

Open the completed design in KLayout:

```bash
make librelane-klayout
```

## Third-party IP

This project uses Jan Schiefer's [`verilog_spi`](https://github.com/janschiefer/verilog_spi), licensed under the GNU LGPL v2.1.

The local `spi_module.v` version is based on the [`jasonwaseq/verilog_spi`](https://github.com/jasonwaseq/verilog_spi) fork and includes cleanup, lint-related changes, and a non-master fallback that drives safe defaults for `SCLK_OUT` and `SS_OUT` when `SPI_MASTER == 0`.

## Template origin

This repository is based on the wafer.space GF180MCU project template. μTheia targets the default `1x1` wafer.space slot and the provided GF180MCU configuration used by the LibreLane flow in this repository. Other slot sizes and alternate template library combinations are not maintained for this design.

## Related Repositories and References

- [wafer.space GF180MCU project template](https://github.com/wafer-space/gf180mcu-project-template)
- [VLSIDA GF180MCU 3.3 V template](https://github.com/VLSIDA/gf180mcu-project-template/tree/3v3-libraries)
- [Google GF180MCU PDK](https://github.com/google/gf180mcu-pdk/)
- [GF180MCU SRAM Forge](https://github.com/mithro/gf180mcu-sram-forge)
- [Prophesee EVT2 format documentation](https://docs.prophesee.ai/stable/data/encoding_formats/evt2.html)
- [FPGA DVS Gesture Classifier](https://github.com/jasonwaseq/FPGA-DVS-Gesture-Classifier)
- [GenX320 STM32F746G-DISCO custom firmware](https://github.com/dolphin-530/x320-stm-usb)

<img width="340" height="340" alt="event_based_visualization" src="https://github.com/user-attachments/assets/8b585ce9-d9c9-4003-8c29-1b3dd0acb709" />


