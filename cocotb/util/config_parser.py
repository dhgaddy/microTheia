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


def _strip_comments(line):
    """Remove # comments."""
    return line.split("#", 1)[0].strip()


def _parse_value(val, params):
    """Parse int or string values."""
    val = val.strip()

    # quoted string
    if val.startswith('"') and val.endswith('"'):
        return val.strip('"')

    # numeric expression (allow previous params)
    try:
        return eval(val, {}, params)
    except Exception:
        try:
            return int(val.replace("_", ""))
        except Exception:
            return val


def load_config(module_name=None):
    params = {}

    if "SIM_CONFIG" not in os.environ:
        return Config(params, module_name)

    cfg_path = os.environ["SIM_CONFIG"]

    with open(cfg_path) as f:
        for raw in f:
            line = _strip_comments(raw)

            if not line:
                continue

            if "=" not in line:
                continue

            key, val = line.split("=", 1)
            key = key.strip()
            val = val.strip()

            params[key] = _parse_value(val, params)

    return Config(params, module_name)


def get_module_params(module_name, src_dir="src"):
    cfg = load_config(module_name)

    src_file = os.path.join(src_dir, f"{module_name}.sv")
    if not os.path.exists(src_file):
        return ""

    with open(src_file) as f:
        text = f.read()

    # match SystemVerilog parameter declarations
    param_names = set(
        re.findall(
            r'parameter(?:\s+(?:signed|unsigned))?'
            r'(?:\s+[A-Za-z_][A-Za-z0-9_]*)?'
            r'(?:\s*\[[^\]]+\])?\s+'
            r'([A-Za-z_][A-Za-z0-9_]*)\s*=',
            text,
        )
    )

    overrides = []

    for k, v in cfg.items():
        if k not in param_names:
            continue

        if isinstance(v, str):
            overrides.append(f'-P{module_name}.{k}="{v}"')
        else:
            overrides.append(f"-P{module_name}.{k}={v}")

    return " ".join(overrides)


if __name__ == "__main__":
    import sys

    if len(sys.argv) != 2:
        print("")
    else:
        print(get_module_params(sys.argv[1]))
