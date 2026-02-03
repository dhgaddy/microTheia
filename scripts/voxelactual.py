#run this file through the command ./voxelactual input.txt

import numpy as np 

IMG_SIZE = 128
BIN_SIZE = 8
GRID = 16

TIMESTAMPS_PER_FRAME = 150000
FRAMES_PER_WINDOW = 8
WINDOWS = 5

def compress_coord(v): #compresses 128x128 down to a 16x16 gridwhere each reduced pixel stores 64 (BIN_SIZE^2) original pixels worth of data
    return min(v // BIN_SIZE, GRID-1)


def classify_motion(frames): #classifies the motion type based off of mat mul
    up = down = left = right = 0

    for t in range(FRAMES_PER_WINDOW - 1):
        A = frames[t]
        B = frames[t+1]

        down  += np.sum(A[:-1, :] * B[1:, :]) #classification math multiplying adjacent rows/columns to determine different weight
        up    += np.sum(A[1:, :]  * B[:-1, :]) #subject to change if needed
        right += np.sum(A[:, :-1] * B[:, 1:])
        left  += np.sum(A[:, 1:]  * B[:, :-1])

    raw_scores = {
        "UP": up,
        "DOWN": down,
        "LEFT": left,
        "RIGHT": right
    }

    signed_scores = {
        "UP":    up - down, #finding the most significant weight
        "DOWN":  down - up,
        "LEFT":  left - right,
        "RIGHT": right - left
    }

    direction = max(signed_scores, key=signed_scores.get)

    return direction, signed_scores, raw_scores

class EventProcessor:

    def __init__(self):
        self.current_frame = np.zeros((16,16), dtype=np.int32) #stores current frame

        self.frames = [] #stores a group of FRAMES_PER_WINDOW frames
        self.windows = [] #stores a group of WINDOWS windows
        self.allwindows = [] #doesnt really matter with our new idea, but originally stored many groups of windows

        self.last_ts = None


    def process_event(self, x, y, ts): #determines when to reset frames and stores compressed pixel data
        if self.last_ts is not None and ts < self.last_ts:
            self.finish_timestamp_block()

        self.last_ts = ts

        bx = compress_coord(x)
        by = compress_coord(y)

        self.current_frame[by, bx] += 1 #heart of the event processing, could have a check at the end of each frame to see if the compressed pixel exceeded a certain value

        if ts >= TIMESTAMPS_PER_FRAME - 750: #my timestamp wouldnt reset unless i changed this to -750. Basically just checks if a timestamp is starting to exceed the bounds 
                                             #of a frame.
            self.finish_timestamp_block()


    def finish_timestamp_block(self): #function for compacting and storing frames/windows

        self.frames.append(self.current_frame.copy())
        self.current_frame[:] = 0

        if len(self.frames) == FRAMES_PER_WINDOW: #if frames reaches the max compress them into one window
            win = np.array(self.frames)

            self.windows.append(win)
            self.allwindows.append(win)

            self.frames = [] #reset frames

            if len(self.windows) == WINDOWS:
                self.process_full_buffer() #process full buffer does the math for gesture classification
                self.windows = []


    def process_full_buffer(self):

        print("===== 5 Windows Ready =====")

        for i, win in enumerate(self.windows):
            direction, signed, raw = classify_motion(win)
            print(f"Window {i}: {direction}  signed={signed}")

def run_from_file(filename): #dont worry about this function it just takes in aedat file data

    acc = EventProcessor()

    with open(filename, "r") as f:
        for line in f:
            if line.strip() == "":
                continue

            x, y, pol, ts = map(int, line.split())
            ts15 = ts & 0x7FFF
            acc.process_event(x, y, ts15)

    all_dirs = []

    for win in acc.allwindows:
        d, _, _ = classify_motion(win)
        all_dirs.append(d)

    if len(all_dirs) == 0:
        print("No windows detected.")
        return

    final = max(set(all_dirs), key=all_dirs.count)

    print("\n============================")
    print("Final Gesture:", final)
    print("============================")

if __name__ == "__main__":
    import sys

    if len(sys.argv) != 2:
        print("Usage: python voxel.py events.txt")
        exit(1)

    run_from_file(sys.argv[1])
