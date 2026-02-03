#run this file through the command ./voxelactual input.txt

import numpy as np 
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

IMG_SIZE = 128
BIN_SIZE = 8
GRID = 16

TIMESTAMPS_PER_FRAME = 35000
FRAMES_PER_WINDOW = 16
WINDOWS = 1


def compress_coord(v): #compresses 128x128 down to a 16x16 grid where each reduced pixel stores 64 (BIN_SIZE^2) original pixels worth of data
    return min(v // BIN_SIZE, GRID-1)

def show_window_grid(win, title="Window"):
    fig, axes = plt.subplots(2, 8, figsize=(16, 4))

    vmax = np.max(win)
    if vmax == 0:
        vmax = 1

    idx = 0
    for r in range(2):
        for c in range(8):
            ax = axes[r][c]
            ax.imshow(win[idx], cmap="gray", vmin=0, vmax=vmax)
            ax.set_title(f"F{idx}")
            ax.axis("off")
            idx += 1

    plt.suptitle(title)
    plt.tight_layout()
    plt.show()


def classify_motion(frames): #classifies the motion type based off of mat mul
    up = down = left = right = 0
    for t in range(FRAMES_PER_WINDOW - 1):
        A = frames[t]
        B = frames[t + 1]
        rows, cols = A.shape
        for i in range(rows):
            for j in range(cols):
                a = A[i, j]
                if a == 0:
                    continue
                if i > 0:
                    up += a * B[i-1, j]
                if i < rows - 1:
                    down += a * B[i+1, j]
                if j > 0:
                    left += a * B[i, j-1]
                if j < cols - 1:
                    right += a * B[i, j+1]

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
        self.current_frame = np.zeros((GRID,GRID), dtype=np.int32) #stores current frame
        self.event_count = 0
        self.frames = [] #stores a group of FRAMES_PER_WINDOW frames
        self.windows = [] #stores a group of WINDOWS windows
        self.allwindows = [] #doesnt really matter with our new idea, but originally stored many groups of windows
        self.x_counter = np.zeros(IMG_SIZE, dtype=np.int32)  # counts how many times each x coordinate occurs
        self.y_counter = np.zeros(IMG_SIZE, dtype=np.int32)  # counts how many times each x coordinate occurs
        self.last_ts = None


    def process_event(self, x, y, pol, ts): #determines when to reset frames and stores compressed pixel data
        self.last_ts = ts
        self.x_counter[x] += 1  # keep track of raw x occurrences
        self.y_counter[y] += 1
        bx = compress_coord(x)
        by = compress_coord(y)
        self.event_count += 1
        if pol == 1:
            self.current_frame[by, bx] += 1 #heart of the event processing, could have a check at the end of each frame to see if the compressed pixel exceeded a certain value
        else:
            if self.current_frame[by, bx] > 0:
                self.current_frame[by, bx] -= 1
        #self.current_frame[by, bx] += 1
        if self.event_count >= TIMESTAMPS_PER_FRAME:
            self.finish_timestamp_block()
            self.event_count = 0

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
            show_window_grid(win, title=f"Window {i}")

            direction, signed, raw = classify_motion(win)
            print(f"Window {i}: {direction}  signed={signed}")

def run_from_file(filename): #dont worry about this function it just takes in aedat file data

    acc = EventProcessor()

    with open(filename, "r") as f:
        for line in f:
            if line.strip() == "":
                continue

            x, y, pol, ts = map(int, line.split())
            ts15 = ts & (TIMESTAMPS_PER_FRAME - 1)
            acc.process_event(x, y, pol, ts15)

    all_dirs = []

    total_motion = {
    "UP": 0,
    "DOWN": 0,
    "LEFT": 0,
    "RIGHT": 0
    }

    for win in acc.allwindows:
        _, signed, _ = classify_motion(win)
    for k in total_motion:
        total_motion[k] += signed[k]

    final = max(total_motion, key=total_motion.get)

    print("\n============================")
    print("Final Gesture:", final)
    print("============================")

    # # optional: plot a histogram
    plt.figure(figsize=(10,4))
    plt.bar(np.arange(IMG_SIZE), acc.x_counter)
    plt.xlabel("X coordinate")
    plt.ylabel("Event count")
    plt.title("Event count per X coordinate")
    plt.show()
    # # optional: plot a histogram
    plt.figure(figsize=(10,4))
    plt.bar(np.arange(IMG_SIZE), acc.y_counter)
    plt.xlabel("Y coordinate")
    plt.ylabel("Event count")
    plt.title("Event count per Y coordinate")
    plt.show()


if __name__ == "__main__":
    import sys

    if len(sys.argv) != 2:
        print("Usage: python voxel.py events.txt")
        exit(1)

    run_from_file(sys.argv[1])
