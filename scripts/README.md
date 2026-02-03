# Running Scripts

## Voxel Implementation
Open aedatconvert.py, change the parameters TS_MIN & TS_MAX to the timestamp range of the gestures that need to be classified 
Run:
 python3 ./aedatconvert input.aedat out.txt

- Next open voxelactual.txt.
- Change the TIMESTAMPS_PER_FRAME parameter to the number of lines in out.txt divided 16, then round down to the nearest whole number.
- run ./voxelactual input.txt
## aedet visualizer 
.aedat_visualizer

Run: 
```bash
  python3 aedat_visualizer.py your_file.aedat --realtime --timesurface --tau-ms 100 --scale 6
```

Notes:
- .aedat file must be in the same directory
- real-time to produce a smooth "video."
- timesurface generates a "trailing" effect to help visualize motions
- tau-ms is the exponential decay factor for the trailing effect
- scale is the size of the window for display
