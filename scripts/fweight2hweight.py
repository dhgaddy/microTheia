import sys

def read_floats(filename):
    values = []
    with open(filename, 'r') as f:
        for line in f:
            for token in line.strip().split():
                try:
                    values.append(float(token))
                except ValueError:
                    pass
    return values


def quantize_to_hex(values):
    if not values:
        return []
    vmin = min(values)
    vmax = max(values)

    if vmax == vmin:
        return ["00"] * len(values)

    step = (vmax - vmin) / 255

    hex_values = []
    for v in values:
        idx = round((v - vmin) / step)
        idx = max(0, min(255, idx))
        hex_values.append(f"{idx:02X}")

    return hex_values


def write_output(filename, hex_values):
    with open(filename, 'w') as f:
        for hv in hex_values:
            f.write(hv + '\n')


def main():
    if len(sys.argv) < 3:
        print("Usage: python script.py input.txt output.txt")
        return

    input_file = sys.argv[1]
    output_file = sys.argv[2]

    values = read_floats(input_file)
    hex_values = quantize_to_hex(values)
    write_output(output_file, hex_values)


if __name__ == "__main__":
    main()