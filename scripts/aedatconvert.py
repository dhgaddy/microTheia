import struct
import sys

HEADER_FMT = "<HHIIIIII"
EVENT_FMT  = "<II"

HEADER_SIZE = struct.calcsize(HEADER_FMT)
EVENT_SIZE  = struct.calcsize(EVENT_FMT)

#runs using ./aedatconvert.py input.aedat out.txt 
TS_MIN = 43310000 #only two lines you need to change
TS_MAX = 51660000 #range gets gesture data from aedat file for only the desired gesture


def skip_text_header(f):
    """Skip lines starting with #"""
    while True:
        pos = f.tell()
        line = f.readline()
        if not line.startswith(b"#"):
            f.seek(pos)
            return


def decode_event(data):
    x = (data >> 17) & 0x1FFF
    y = (data >> 2)  & 0x1FFF
    polarity = (data >> 1) & 0x1
    return x, y, polarity


def convert_aedat(input_file, output_file):

    with open(input_file, "rb") as fin, open(output_file, "w") as fout:

        skip_text_header(fin)

        block_index = 0

        while True:
            header_bytes = fin.read(HEADER_SIZE)
            if len(header_bytes) < HEADER_SIZE:
                break

            (eventType,
             eventSource,
             eventSize,
             tsOffset,
             tsOverflow,
             eventCapacity,
             eventNumber,
             eventValid) = struct.unpack(HEADER_FMT, header_bytes)

            wrote_block_header = False

            for _ in range(eventNumber):
                ev_bytes = fin.read(EVENT_SIZE)
                if len(ev_bytes) < EVENT_SIZE:
                    return

                data, ts = struct.unpack(EVENT_FMT, ev_bytes)

                # -------- TIMESTAMP FILTER --------
                if ts < TS_MIN or ts > TS_MAX:
                    continue
                # ----------------------------------

                if not wrote_block_header:
                    # fout.write(f"\n# Block {block_index}\n") Uncomment this if you want useless data on eventtype and size, from what i found it was always the same
                    # fout.write(
                    #     f"eventType={eventType} "
                    #     f"eventSource={eventSource} "
                    #     f"eventNumber={eventNumber}\n"
                    # )
                    wrote_block_header = True

                x, y, p = decode_event(data)
                fout.write(f"{x} {y} {p} {ts}\n")

            block_index += 1

    print(f"Done. Filtered output written to {output_file}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python aedat_to_text_filtered.py input.aedat output.txt")
        sys.exit(1)

    convert_aedat(sys.argv[1], sys.argv[2])
