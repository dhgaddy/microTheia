import numpy as np
import matplotlib.pyplot as plt
from voxel import EventProcessor
from testvoxel import generate_down_motion   # or paste function here

acc = EventProcessor()
events = generate_down_motion()

for e in events:
    acc.process_event(*e)

acc.finish_timestamp_block()
frames = []
for win in acc.allwindows:
    for f in win:
        frames.append(f)

print("Total frames:", len(frames))

fig, axes = plt.subplots(5, 8, figsize=(16, 10))

idx = 0
for r in range(5):
    for c in range(8):
        ax = axes[r][c]
        img = frames[idx]

        ax.imshow(img, cmap="gray")
        ax.set_title(f"F{idx}")
        ax.axis("off")
        idx += 1

plt.suptitle("16×16 Event Frames (Downward Motion)")
plt.tight_layout()
plt.show()
