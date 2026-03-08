import os
import re


class Config:
    def __init__(self, params, module_name=None):
        self._params = params
        self._module = module_name

    def get(self, name, default=None):
        return self._params.get(name, default)

    def require(self, name):
        if name not in self._params:
            raise RuntimeError(
                f"Parameter '{name}' not found for module '{self._module}'. "
                f"Available: {list(self._params.keys())}"
            )
        return self._params[name]

    def __getitem__(self, key):
        return self.require(key)

    def __contains__(self, key):
        return key in self._params

    def keys(self):
        return self._params.keys()

    def items(self):
        return self._params.items()


def load_config(module_name=None):
    params = {}

    # ---------------- CONFIG FILE MODE ----------------
    if "SIM_CONFIG" in os.environ:
        cfg_path = os.environ["SIM_CONFIG"]

        with open(cfg_path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue

                key, val = line.split("=", 1)
                key = key.strip()
                val = val.strip()

                try:
                    params[key] = eval(val, {}, params)
                except:
                    params[key] = int(val.replace("_", ""))

    # ---------------- CARGS MODE ----------------
    if "SIM_CARGS" in os.environ:
        cargs = os.environ["SIM_CARGS"]

        matches = re.findall(r'-P(\w+)\.(\w+)=(\d+)', cargs)

        for mod, param, value in matches:
            if module_name is None or mod == module_name:
                params[param] = int(value)

    return Config(params, module_name)


def get_module_params(module_name, src_dir="src"):
    cfg = load_config(module_name)

    src_file = os.path.join(src_dir, f"{module_name}.sv")
    if not os.path.exists(src_file):
        return ""

    with open(src_file) as f:
        text = f.read()

    # Support typed/untyped SystemVerilog parameters, e.g.:
    #   parameter FOO = 1
    #   parameter int FOO = 1
    #   parameter [31:0] foo_p = 32
    param_names = set(
        re.findall(
            r'parameter(?:\s+(?:signed|unsigned))?(?:\s+[A-Za-z_][A-Za-z0-9_]*)?(?:\s*\[[^\]]+\])?\s+([A-Za-z_][A-Za-z0-9_]*)\s*=',
            text,
        )
    )

    overrides = []
    for k, v in cfg.items():
        if k in param_names:
            overrides.append(f"-P{module_name}.{k}={v}")

    return " ".join(overrides)


# CLI support (unchanged behavior)
if __name__ == "__main__":
    import sys

    if len(sys.argv) != 2:
        print("")
    else:
        print(get_module_params(sys.argv[1]))
