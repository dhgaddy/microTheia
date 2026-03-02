import os
import re


def load_config():
    cfg_path = os.environ.get("SIM_CONFIG", "configs/voxel_default.txt")
    params = {}

    with open(cfg_path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            key, val = line.split("=")
            key = key.strip()
            val = val.strip()

            # evaluate simple expressions like NUM_CELLS = NUM_BINS * GRID_SIZE * GRID_SIZE
            try:
                params[key] = eval(val, {}, params)
            except:
                params[key] = int(val.replace("_", ""))

    return params


def get_module_params(module_name, src_dir="src"):
    """
    Returns a string of -P overrides for parameters
    that exist in the given module.
    """
    cfg = load_config()

    src_file = os.path.join(src_dir, f"{module_name}.sv")
    if not os.path.exists(src_file):
        return ""

    with open(src_file) as f:
        text = f.read()

    # Only match 'parameter', not 'localparam'
    param_names = set(
        re.findall(r'parameter\s+([A-Za-z_][A-Za-z0-9_]*)', text)
    )

    overrides = []
    for k, v in cfg.items():
        if k in param_names:
            overrides.append(f"-P{module_name}.{k}={v}")

    return " ".join(overrides)


# Allow CLI usage (so Make can call this cleanly)
if __name__ == "__main__":
    import sys

    if len(sys.argv) != 2:
        print("")
    else:
        print(get_module_params(sys.argv[1]))