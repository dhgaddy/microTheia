import numpy as np
IMG_SIZE = 128
BIN_SIZE = 8
GRID = 16
TIMESTAMPS_PER_FRAME = 32768 * 4 
FRAMES_PER_WINDOW = 10
WINDOWS = 5

def compress_coord(v):
    return min(v // BIN_SIZE, GRID-1)

def classify_motion(frames):
    up = down = left = right = 0
    for t in range(7):
        A = frames[t]
        B = frames[t+1]
        down += np.sum(A[:-1, :] * B[1:, :])
        up += np.sum(A[1:, :]  * B[:-1, :])
        right += np.sum(A[:, :-1] * B[:, 1:])
        left += np.sum(A[:, 1:]  * B[:, :-1])
    scores = {
        "UP": up,
        "DOWN": down,
        "LEFT": left,
        "RIGHT": right
    }
    actualscore = {
        "UP":    up - down,
        "DOWN":  down - up,
        "LEFT":  left - right,
        "RIGHT": right - left
    }

    direction = max(actualscore, key=actualscore.get)
    highestdir = actualscore[direction]
    return highestdir, scores

class EventProcessor:
    def __init__(self):
        self.current_frame = np.zeros((16,16), dtype=np.int32)
        self.frames = []
        self.windows = []
        self.allwindows = []
        self.last_ts = None
        self.ts_counter = 0

    def process_event(self, x, y, pol, ts):
        if self.last_ts is not None and ts < self.last_ts:
            self.finish_timestamp_block()

        self.last_ts = ts
        bx = compress_coord(x)
        by = compress_coord(y)

        self.current_frame[by, bx] += 1
        if ts >= TIMESTAMPS_PER_FRAME-1:
            self.finish_timestamp_block()

    def finish_timestamp_block(self):
        self.frames.append(self.current_frame.copy())
        self.current_frame[:] = 0
        self.ts_counter = 0

        if len(self.frames) == FRAMES_PER_WINDOW:
            win = np.array(self.frames)
            self.windows.append(win)
            self.allwindows.append(win)
            self.frames = []
            if len(self.windows) == WINDOWS:
                self.process_full_buffer()
                self.windows = []

    def process_full_buffer(self):
        print("Done")

        for i, win in enumerate(self.windows):
            direction = classify_motion(win)
            print(f" Window {i} motion:", direction)


