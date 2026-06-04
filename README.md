# μTheia
μTheia is a GF180MCU event-based machine vision ASIC for motion-pattern classification from EVT2 event streams. The chip receives EVT2 event data and configuration commands over SPI, decodes timestamped events, compresses 320×320 sensor coordinates into a 16x16 spatial grid, bins events into 16 programmable-duration temporal bins, stores feature windows in SRAM, performs integer MAC scoring against programmable class weights, and reports pattern detection for the 4 programmable classes through SPI or selectable physical debug/output pins.

<img width="657" height="855" alt="image" src="https://github.com/user-attachments/assets/28d2bab8-3486-4232-8110-f9bdbcd9f0cf" />

Project uses wafer.space MPW and runs using the gf180mcu PDK.

## Documentation

Full technical documentation is in the [`docs/`](docs/) directory:

| Document | Contents |
|----------|----------|
| [`docs/architecture.md`](docs/architecture.md) | System architecture, block diagram, pipeline stages, memory map, control flow |
| [`docs/rtl_reference.md`](docs/rtl_reference.md) | Per-module RTL reference: ports, parameters, behavior, timing |
| [`docs/simulation.md`](docs/simulation.md) | cocotb testbenches, utility functions, configuration files, waveform viewing |
| [`docs/limitations.md`](docs/limitations.md) | Known limitations, things that don't work, results, and future improvements |

The [`debug_mux_pinout.txt`](debug_mux_pinout.txt) file documents the per-bit assignments for all 11 debug bus pages.

## Caveats

> [!IMPORTANT]
> Read these before running the project for the first time.

- **Nix/OpenROAD first-run compile time.** The `flake.nix` environment compiles OpenROAD from source (the `leo/gf180mcu` branch) on first `nix-shell` invocation. This takes **30–90 minutes** depending on CPU speed. The result is cached locally; subsequent shells start in seconds.

- **Gate-level simulation is slow.** `make sim-gl` runs a single full-chip GL simulation and takes approximately **7 hours** on a modern workstation. `make sim-gl-parallel` runs four gesture tests simultaneously; ensure you have adequate RAM and CPU cores before starting.

- **Waveform file size.** GL simulation FST waveform files can reach **tens of gigabytes**. Ensure at least 50 GB of free disk space before running gate-level or chip-top RTL simulations.

- **Weight loading is required before inference.** After every power cycle, the chip starts in `BOOT` state and will not classify events until a full weight/threshold load completes (`BOOT_REQ` → weight commands → `EVT_READS_DONE`). If the SPI master loses sync during loading, a fresh load from the beginning is required.

- **`make sim` requires DUT.** Running `make sim` without `DUT=<module_name>` will fail. Always specify the module, e.g. `make sim DUT=voxel_bin_core`.

- **PDK must be cloned before simulation.** Run `make clone-pdk` once to download the GF180MCU PDK. Simulation will fail without it because the SRAM behavioral models are part of the PDK.

- **SDF back-annotation is not fully working.** `make sim-sdf` is provided but timing-accurate simulation is not reliable because the Verilog timing models for GF180MCU SRAMs are not consistently present in the open PDK. The simulation will fall back silently to zero-delay functional mode.

- **iCE40 FPGA target uses a reduced configuration.** The full 16×16 × 16-bin design does not fit in iCE40 resources. The FPGA flow uses an 8×8 grid with 8 bins. See [`ice40/README.md`](ice40/README.md) for details.

- **cocotb 2.0 API only.** Testbenches use the cocotb 2.0 coroutine syntax (`async def`, no `@cocotb.coroutine`). Do not mix 1.x-style testbench code; it will error at import time.

## Dependencies

### Nix

Too manage all dependencies, the project template includes a Nix shell with all the required tools.
Install Nix and LibreLane by following the Nix-based installation instructions: https://librelane.readthedocs.io/en/latest/installation/nix_installation/index.html
To activate the shell, simply run `nix-shell` in the root directory of this repository. The subsequent steps assume that you are in the Nix shell of the project template.

### Docker

Ensure [Docker](https://www.docker.com/) is installed and start a devcontainer. You can also open this repository in a github codespace.

## Prerequisites and setup

Make sure **Git** and **Git LFS** are installed.

```bash
git clone git@github.com:dolphin-530/microTheia.git
cd microTheia
make config-pdk
nix-shell
make sim
make librelane
make sim-gl
make sim-sdf # (like sim-gl but with SDF back-annotated)
```

The project uses the open_pdks gf180mcuD variant of the PDK.
To clone the latest PDK version via [Ciel](https://github.com/fossi-foundation/ciel), run `make clone-pdk`.

## Implement the Design

This repository contains a Nix flake that provides a shell with the [`leo/gf180mcu`](https://github.com/librelane/librelane/tree/leo/gf180mcu) branch of LibreLane.

Simply run 
```bash
export NIX_PATH=nixpkgs=https://github.com/NixOS/nixpkgs/archive/nixos-25.05.tar.gz
nix-shell
```
in the root of this repository.

> [!NOTE]
> Since we are working on a branch of LibreLane, OpenROAD needs to be compiled locally. This will be done automatically by Nix, and the binary will be cached locally. 

With this shell enabled, run the implementation:

```bash
make librelane
```

## View the Design

After completion, you can view the design using the OpenROAD GUI:

```bash
make librelane-openroad
```

Or using KLayout:

```bash
make librelane-klayout
```

## Copying the Design to the Final Folder

To copy your latest run to the `final/` folder in the root directory of the repository, run the following command:

```bash
make copy-final
```

This will only work if the last run was completed without errors.

## Verification and Simulation

We use [cocotb](https://www.cocotb.org/), a Python-based testbench environment, for the verification of the chip.
The underlying simulator is Icarus Verilog (https://github.com/steveicarus/iverilog).

The testbenchs are located in `cocotb`. To run the RTL simulation, run the following command:

```bash
make sim DUT=module_name CONFIG=config_name
```

If DUT isn't provided it fails, if CONFIG isn't provided it will default to the compile arguments in the Makefile. Configs are stored in the configs folder.

To simulate all, run

```bash
make sim-all
```

To run the GL (gate-level) simulation, run the following command:

```bash
make sim-gl
```

> [!NOTE]
> You need to have the latest implementation of your design in the `final/` folder. After implementing the design, execute 'make copy-final' to copy all necessary files.

In both cases, a waveform file will be generated under `cocotb/sim_build/chip_top.fst`.
You can view it using a waveform viewer, for example, [GTKWave](https://gtkwave.github.io/gtkwave/) and there is [Surfer](https://gitlab.com/surfer-project/surfer) installed within the devcontainer.

```bash
make sim-view
```

You can now update the testbench according to your design.

## Synthesis for ICE40 FPGA and communicating with it

The current architecture we are using is voxel_bin.

```bash
make ice40 ARCH=architecture     # Run iCE40 FPGA build
make ice40-prog                  # Program iCE40 board
make ice40-timing                # Timing report for iCE40 build
make ice40-clean                 # Cleans out all ice40 logic
```

Once synthesized and having a working bitstream to flash and test, go into the [`ice40`](ice40/README.md) folder.

## Tool Versions

| Tool | Version | Source |
|------|---------|--------|
| Icarus Verilog | (from flake.nix) | Nix |
| OpenROAD | (compiled locally) | Nix |
| LibreLane | leo/gf180mcu branch | GitHub |
| oss-cad-suite | 2024-11-21 | Dockerfile |
| sv2v | 0.0.13 | Dockerfile |
| numpy | 2.4.3 | Dockerfile, scripts/requirements.txt, ice40/requirements.txt |
| open-cv-python | 4.13.0.92 | Dockerfile, scripts/requirements.txt, ice40/requirements.txt |
| matplotlib | 3.10.8 | Dockerfile, scripts/requirements.txt |
| cocotb | 2.0.1 | Dockerfile, scripts/requirements.txt |
| pyserial | 3.5 | Dockerfile, ice40/requirements.txt |

Run `pip install -r scripts/requirements.txt` for RTL simulation, or `pip install -r ice40/requirements.txt` for FPGA tools.

## Third Party
This project uses an SPI module from:

- Jan Schiefer, "verilog_spi"
  https://github.com/janschiefer/verilog_spi

Licensed under the GNU LGPL v2.1.

The functional change we have made in spi_module.v was adding a non-master fallback in the generate block; previously SCLK_OUT/SS_OUT were only assigned inside if (SPI_MASTER), and now there is an else branch that forces safe defaults when SPI_MASTER == 0 (SCLK_OUT = 1'b0, SS_OUT = 1'b1). There is also clean up and verilator lint flags to reduce lint errors. The changes are saved to a fork [here](https://github.com/jasonwaseq/verilog_spi).

## Choosing a Different Slot Size

The design utilizes the default slot size of `1x1` however this can be changed, although would require lots of effort and a reduction in scope for the design.

The template supports the following slot sizes: `1x1`, `0p5x1`, `1x0p5`, `0p5x0p5`.
By default, the design is implemented using the `1x1` slot definition.

To select a different slot size, simply set the `SLOT` environment variable.
This can be done when invoking a make target:

```bash
SLOT=0p5x0p5 make librelane
```

Alternatively, you can export the slot size:

```bash
export SLOT=0p5x0p5
```

You can change the slot that is selected by default in the Makefile by editing the value of `DEFAULT_SLOT`.

## Select Different IP Libraries

The project template has support for selecting libraries with the below environment variables:

| Env  | Available Values                                                          | Description                |
|------|---------------------------------------------------------------------------|----------------------------|
| SCL  | gf180mcu_fd_sc_mcu7t5v0, gf180mcu_fd_sc_mcu9t5v0, gf180mcu_as_sc_mcu7t3v3 | The standard cell library. |
| PAD  | gf180mcu_fd_io, gf180mcu_ocd_io                                           | The I/O pad library.       |
| SRAM | gf180mcu_fd_ip_sram, gf180mcu_ocd_ip_sram                                 | The SRAM library.          |

For example, to build the 0p5x0p5 chip with 3v3 libraries:

```bash
SLOT=0p5x0p5 SCL=gf180mcu_as_sc_mcu7t3v3 PAD=gf180mcu_ocd_io SRAM=gf180mcu_ocd_ip_sram make librelane
```

The default values can be changed in the Makefile.

> [!NOTE]
> Not all of the community-created IPs have been tested yet, so support for them is experimental!

## Building a Standalone Padring for Analog Design

To build just the padring without any standard cell rows, digital routing or filler cells, run the following command:

```bash
make librelane-padring
```

It is also possible to build the padring for other slot sizes:

```bash
SLOT=0p5x0p5 make librelane-padring
```

## Precheck

To check whether the design is suitable for manufacturing, run the [gf180mcu-precheck](https://github.com/wafer-space/gf180mcu-precheck) with the layout.


## Notes

- For more comprehensive SystemVerilog support, enable the `USE_SLANG` variable in the LibreLane configuration.
- https://github.com/wafer-space/gf180mcu-project-template
- https://github.com/VLSIDA/gf180mcu-project-template/tree/3v3-libraries
- https://github.com/jasonwaseq/FPGA-DVS-Gesture-Classifier
- https://github.com/jasonwaseq/Verilog-Memory-Hardware
- https://github.com/jasonwaseq/GenX320_STM32F746G-DISCO
- https://github.com/google/gf180mcu-pdk/
- https://github.com/mithro/gf180mcu-sram-forge
- [Event Camera Clips](https://drive.google.com/drive/folders/1kUSThZpBVr_RSmRtKbDS8sVFCjakwOAj?usp=sharing)
- https://docs.prophesee.ai/stable/data/encoding_formats/evt2.html
- https://docs.prophesee.ai/stable/data/encoding_formats/evt3.html
- https://docs.google.com/spreadsheets/d/1fW5ecBsLSec4hXBMaOjMUHQGslm4y-QUILgrxqS8MpA/edit?gid=0#gid=0
