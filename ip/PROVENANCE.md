# IP Provenance

This directory contains third-party IP blocks copied directly into the repository.
The table below records the origin of each block for provenance tracking purposes.

> **Note on .gitmodules:** These IPs are vendored copies rather than git submodules.
> Upstream URLs and commit references are recorded here to preserve traceability.

## Included IPs

| Directory | Name | Source | Upstream URL | Notes |
|-----------|------|--------|-------------|-------|
| `gf180mcu_ws_ip__id/` | wafer.space ID macro | wafer.space | https://github.com/wafer-space/gf180mcu_ws_ip__id | Copied at project creation |
| `gf180mcu_ws_ip__logo/` | wafer.space Logo macro | wafer.space | https://github.com/wafer-space/gf180mcu_ws_ip__logo | Copied at project creation |

## License Status

The wafer.space IP blocks (`gf180mcu_ws_ip__id`, `gf180mcu_ws_ip__logo`) do **not** include
a license file in this copy. Their distribution terms are not explicitly stated.

**Action required:** Contact wafer.space to confirm the license under which these IPs
are distributed for use in MPW submissions, and add the appropriate license file.
See [LICENSE.waferspace.md](LICENSE.waferspace.md) for more details.

## Other Third-Party Components (Not in This Directory)

The following dependencies are fetched at build time and are **not** vendored here:

| Component | Source | License | Inclusion Method |
|-----------|--------|---------|-----------------|
| GF180MCU PDK (wafer.space fork) | https://github.com/wafer-space/gf180mcu | Apache-2.0 | `make clone-pdk` |
| LibreLane | https://github.com/librelane/librelane | Apache-2.0 | Nix flake |
| cocotb | https://cocotb.org | BSD-3-Clause | pip |
| Icarus Verilog | https://github.com/steveicarus/iverilog | GPL-2.0 | External tool |
| oss-cad-suite | https://github.com/YosysHQ/oss-cad-suite-build | Various | External tool |
