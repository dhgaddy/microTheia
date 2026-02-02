import numpy as np
from voxel import classify_motion
from voxel import EventProcessor

#run ./testvoxel.py to see the weight classifications from voxel.py. run ./avoxel.py to animate what is being tested
#voxel.py uses only test data, voxelactual.py reads from converted aedat files.
tsmod = 4096 
psize = 10 #change this to adjust the size of compressed pixels
def generate_down_motion():
    events = [] #makes 5 windows, 8 frames per window, 8 * tsmod timestamps per frame. incrmeents to ~120 on xpos and ypos
    base_x = 40
    county = 0
    y_pos = -8
    x_pos = -8
    for window in range(5):
        county += 1
        for frame in range(8):
            y_pos += 3 #change to += 8 if you want to test on 320x320, otherwise this works for 128x128
            x_pos += 3
            for ts in range(8 * tsmod):
                if ts % tsmod == 0:
                    for p in range(psize):
                        events.append((base_x+p, y_pos + ts // tsmod, 1, ts)) #downwards motion starting from top middle #1
                        events.append((base_x+p+20, y_pos + ts // tsmod, 1, ts)) #downwards motion starting from top middle #2
                        events.append((x_pos + ts // tsmod, base_x + p, 1, ts)) #rightwards motion from the middle of the screen starting on the far left
                        events.append((127 - x_pos - p - ts // tsmod, 127 - x_pos - p - ts // tsmod, 1, ts)) #top left motion starting on the bottom right
    return events


def run_test(generator, label, log_filename):
    acc = EventProcessor()
    events = generator()

    with open(log_filename, 'w') as f:
        for e in events:
            f.write(f"{e}\n")
            acc.process_event(*e)

    for i, window in enumerate(acc.windows):
        result, scores = classify_motion(window)
        print(f"{label} Window {i}: {result}, scores={scores}")


if __name__ == "__main__":
    #run_test(generate_right_motion, "RIGHT", "right_motion_events.log")
    #run_test(generate_left_motion, "LEFT", "left_motion_events.log")
    run_test(generate_down_motion, "DOWN", "down_motion_events.log")
    #run_test(generate_up_motion, "UP", "up_motion_events.log")
