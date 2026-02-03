## .aedat_visualizer

Run: 
  python3 aedat_visualizer.py your_file.aedat --realtime --timesurface --tau-ms 100 --scale 6
Notes:
.aedat file must be in same directory
--realtime to produce a smooth "video"
--timesurface generates a "trailing" effect to help visualize motions
--tau-ms is exponential decay factor for the trailing effect
--scale is the size of the window for display
