# Group G (Still unsure of the name)

## Setup
To run locally

```bash
git clone git@github.com:MrPoloGit/group-g.git
cd group-g
git submodule update --init --recursive
```

Ensure [Docker](https://www.docker.com/) is installed and start the devcontainer. You can also open this repository in a github codespace.

## Run
```bash
make help           # Show this help message
make lint           # Verify Verilog including generated
make synth          # Synthesize Verilog
make sim            # Simulate Verilog
make chisel-test    # Test Chisel modules
make chisel-verilog # Generates the verilog, choose the file 
make clean-synth    # Removes out all synth files
make clean-chisel   # Removes all generated verilog files
```

## Other repositories

https://github.com/chipsalliance/chisel-template

https://github.com/wafer-space/gf180mcu-project-template

https://github.com/Jilin-Zhang/ASYNC-Chisel
