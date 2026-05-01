# μTheia

Project uses wafer.space MPW and runs using the gf180mcu PDK.

## Prerequisites and setup

Make sure Git and Git LFS are installed.

```bash
git clone git@github.com:MrPoloGit/group-g.git
cd group-g
make clone-pdk
git submodule update --init --recursive
git lfs pull
```

Ensure [Docker](https://www.docker.com/) is installed and start the devcontainer. You can also open this repository in a github codespace.

> [!NOTE]
> We use a custom fork of the [gf180mcuD PDK variant](https://github.com/wafer-space/gf180mcu) until all changes have been upstreamed.

To clone the latest PDK version, simply run `make clone-pdk`.

In the next step, install LibreLane by following the Nix-based installation instructions: https://librelane.readthedocs.io/en/latest/installation/nix_installation/index.html

## Implement the Design

This repository contains a Nix flake that provides a shell with the [`leo/gf180mcu`](https://github.com/librelane/librelane/tree/leo/gf180mcu) branch of LibreLane.

Simply run `nix-shell` in the root of this repository.

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

## Choosing a Different Slot Size

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

## Pin Assignment (INCOMPLETE)

| Signal | Pins | Direction | Type | Notes |
|--------|------|-----------|------|-------|
| clk | 1 | in | Clock | System clock |
| rst | 1 | in | Reset | Active high |
| event_data[7:0] | 8 | in | Data | EVT2.0 events (32-bit over 4 cycles) |
| event_valid | 1 | in | Handshake | Data valid |
| event_ready | 1 | out | Handshake | Ready to accept |
| spi_clk, spi_mosi, spi_miso, spi_cs | 4 | in/out | SPI | Configuration interface |
| uart_tx, uart_rx | 2 | in/out | UART | Debug/output |
| debug[N:0] | M | out | Debug | Tied to VSS when not used |

## Third Party
- https://github.com/google/globalfoundries-pdk-ip-gf180mcu_fd_ip_sram

This project uses an SPI module from:

- Jan Schiefer, "verilog_spi"
  https://github.com/janschiefer/verilog_spi

Licensed under the GNU LGPL v2.1.

## Precheck

To check whether our design is suitable for manufacturing, run the [gf180mcu-precheck](https://github.com/wafer-space/gf180mcu-precheck) with the layout.

## Notes

### General
- For more comprehensive SystemVerilog support, enable the `USE_SLANG` variable in the LibreLane configuration.
- https://github.com/chipsalliance/chisel-template
- https://github.com/wafer-space/gf180mcu-project-template
- https://github.com/Jilin-Zhang/ASYNC-Chisel
- https://github.com/jasonwaseq/FPGA-DVS-Gesture-Classifier
- https://github.com/jasonwaseq/Verilog-Memory-Hardware
- https://github.com/jasonwaseq/GenX320_STM32F746G-DISCO
- https://github.com/google/gf180mcu-pdk/
- [GF180MCU Tutorial - Single Video](https://www.youtube.com/watch?v=USCmZuREMTE)
- https://github.com/mithro/gf180mcu-sram-forge
- [Event Camera Clips](https://drive.google.com/drive/folders/1kUSThZpBVr_RSmRtKbDS8sVFCjakwOAj?usp=sharing)
- https://github.com/gcohen/AMOS-Short-Course
- https://docs.prophesee.ai/stable/data/encoding_formats/evt2.html
- https://docs.prophesee.ai/stable/data/encoding_formats/evt3.html
- https://docs.google.com/spreadsheets/d/1fW5ecBsLSec4hXBMaOjMUHQGslm4y-QUILgrxqS8MpA/edit?gid=0#gid=0
