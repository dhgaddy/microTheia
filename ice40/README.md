# DVS Camera Emulator Tools

This directory contains tools for emulating a Dynamic Vision Sensor (DVS) using a standard webcam, enabling testing of the DVS Gesture Accelerator without actual DVS hardware.

```bash
python tools/dvs_camera_emulator.py --port COM3 --baud 115200 --arch auto --preview --no-noise --contrast 0.20 --max-events 86
```

## Overview

Dynamic Vision Sensors (DVS) are neuromorphic cameras that detect changes in pixel intensity asynchronously, outputting events only when brightness changes beyond a threshold. This is fundamentally different from traditional frame-based cameras.

### How Real DVS Sensors Work

Real DVS pixels operate independently and asynchronously:
1. Each pixel maintains a **reference voltage** proportional to log-intensity
2. When current log-intensity differs from reference by a threshold θ, an event fires
3. **Only the pixel that fires** updates its reference (not neighboring pixels)
4. A **refractory period** prevents immediate re-firing of the same pixel
5. Events have **microsecond-level timestamps** (true asynchronous operation)

### How This Emulator Models DVS Behavior

This emulator converts webcam frames into realistic DVS-like events by:
1. Computing log intensity of each frame
2. Maintaining a **per-pixel reference memory** (like real DVS photoreceptors)
3. Detecting pixels where log-intensity change exceeds the contrast threshold
4. Generating ON events (brightness increased) and OFF events (brightness decreased)
5. Updating reference **only for pixels that fired** (realistic behavior)
6. Applying **refractory period** to prevent rapid re-firing
7. Optionally adding **background noise** (shot noise, hot pixels, reference leak)

## Realism Features

| Feature | Description | Real DVS Behavior |
|---------|-------------|-------------------|
| Per-pixel reference | Each pixel tracks its own reference intensity | ✓ Matches DVS photoreceptor memory |
| Contrast threshold | Log-domain threshold (~15% intensity change) | ✓ Matches DVS temporal contrast |
| Selective reference update | Only pixels that fire update their reference | ✓ Critical for accurate edge detection |
| Refractory period | Minimum 1ms between events at same pixel | ✓ Models DVS pixel reset time |
| Reference leak | Reference slowly drifts over time | ✓ Models capacitor leak in real pixels |
| Shot noise | Random background events | ✓ Models photocurrent noise |
| Hot pixels | Some pixels have elevated noise | ✓ Models manufacturing defects |

## Files

| File | Description |
|------|-------------|
| `dvs_camera_emulator.py` | Main emulator - captures webcam and generates DVS events in real-time |
| `dvs_event_player.py` | Replays saved event files to FPGA |

## Installation

```bash
# Ubuntu/Debian - install all dependencies system-wide
sudo apt install python3-opencv python3-serial python3-numpy

# That's it! No virtual environment needed.
```

For other systems (Windows/macOS), use pip:
```bash
pip install -r requirements.txt
```

## Usage

### Quick Start Examples

**Simulation mode (no camera required - great for testing):**
```bash
python dvs_camera_emulator.py --simulate --preview
```
Press keys 1-4 to trigger gestures (UP/DOWN/LEFT/RIGHT).

**Preview mode with camera:**
```bash
python dvs_camera_emulator.py --preview
```

**Send events to FPGA:**
```bash
# Linux
python dvs_camera_emulator.py --port /dev/ttyUSB0 --preview

# Windows  
python dvs_camera_emulator.py --port COM3 --preview

# macOS
python dvs_camera_emulator.py --port /dev/cu.usbserial-* --preview
```

### Input Sources

**Use webcam (default):**
```bash
python dvs_camera_emulator.py --preview
python dvs_camera_emulator.py --camera 1 --preview  # Use camera ID 1
```

**Use a video file as input:**
```bash
python dvs_camera_emulator.py --video gesture_video.mp4 --preview
python dvs_camera_emulator.py --video gesture_video.mp4 --loop --preview  # Loop playback
```

**Simulation mode (synthetic gestures):**
```bash
python dvs_camera_emulator.py --simulate --preview
```

### Threshold & Sensitivity

The emulator supports two threshold modes:

**Realistic contrast threshold (default):**
```bash
# Default 15% contrast threshold (0.15 in log domain)
python dvs_camera_emulator.py --preview

# More sensitive (detects ~10% intensity changes)
python dvs_camera_emulator.py --contrast 0.10 --preview

# Less sensitive (detects ~25% intensity changes)
python dvs_camera_emulator.py --contrast 0.25 --preview
```

**Legacy threshold mode:**
```bash
# Use original frame-difference algorithm
python dvs_camera_emulator.py --legacy-mode --threshold 15 --preview
python dvs_camera_emulator.py --legacy-mode --threshold 10 --preview  # More sensitive
python dvs_camera_emulator.py --legacy-mode --threshold 25 --preview  # Less sensitive
```

### Noise Model

Control the realistic noise simulation:

```bash
# Disable noise for clean output
python dvs_camera_emulator.py --no-noise --preview

# Adjust noise parameters
python dvs_camera_emulator.py --leak-rate 0.002 --preview  # More reference drift
python dvs_camera_emulator.py --shot-noise 0.0005 --preview  # More background noise
```

### Output Options

**Save events to file (EVT 2.0 format, default):**
```bash
python dvs_camera_emulator.py --save recording.bin --preview
python dvs_camera_emulator.py --save recording.bin --format evt2 --preview  # explicit
```

**Save events to file (legacy DVS1 format):**
```bash
python dvs_camera_emulator.py --save recording.bin --format dvs1 --preview
```

**Send to FPGA via UART:**
```bash
# Auto-detect architecture (recommended)
python dvs_camera_emulator.py --port /dev/ttyUSB0 --baud 115200 --arch auto --preview --aspect-mode crop --subsample-mode spatial

# Force voxel_bin architecture (echo-based)
python dvs_camera_emulator.py --port /dev/ttyUSB0 --baud 115200 --arch voxel_bin --preview

# Force gradient_map architecture (ASCII gesture output)
python dvs_camera_emulator.py --port /dev/ttyUSB0 --baud 115200 --arch gradient_map --preview

# Windows example
python dvs_camera_emulator.py --port COM3 --baud 115200 --arch auto --preview --aspect-mode crop --subsample-mode spatial
```

**Limit events per frame (prevent UART overflow):**
```bash
# At 115200 baud / 30 FPS, ~86 events/frame is a practical ceiling.
python dvs_camera_emulator.py --port /dev/ttyUSB0 --max-events 86 --preview
```

**Improve webcam robustness (left/right and background clutter):**
```bash
# Preserve geometry (crop), suppress peripheral motion (ROI), spatially fair downsampling
python dvs_camera_emulator.py --port /dev/ttyUSB0 --baud 115200 --max-events 86 --aspect-mode crop --roi-scale 0.75 --subsample-mode spatial --preview
```

### Advanced Options

**Adjust timing:**
```bash
python dvs_camera_emulator.py --fps 60 --preview  # Higher frame rate
python dvs_camera_emulator.py --refractory 2000 --preview  # 2ms refractory period
```

**Noise filtering:**
```bash
python dvs_camera_emulator.py --noise-filter 5 --preview  # Stronger Gaussian blur
python dvs_camera_emulator.py --noise-filter 0 --preview  # Disable blur
```

**Change resolution:**
```bash
python dvs_camera_emulator.py --resolution 128 --preview  # Lower resolution
```

### Complete Options Reference

```bash
python dvs_camera_emulator.py --help
```

| Option | Default | Description |
|--------|---------|-------------|
| `--camera` | 0 | Camera device ID |
| `--video` | None | Video file input instead of camera |
| `--simulate` | False | Simulate gestures without camera |
| `--port` | None | Serial port for UART output |
| `--arch` | auto | FPGA architecture: `auto`, `voxel_bin`, or `gradient_map` |
| `--baud` | 115200 | UART baud rate |
| `--contrast` | 0.15 | Contrast threshold (log-domain, ~15% change) |
| `--threshold` | 15 | Legacy threshold (scaled difference) |
| `--legacy-mode` | False | Use legacy frame-difference algorithm |
| `--refractory` | 1000 | Refractory period in μs |
| `--resolution` | 320 | Output resolution |
| `--fps` | 30 | Target frame rate |
| `--preview` | False | Show preview window |
| `--save` | None | Save events to binary file |
| `--format` | evt2 | Output format: `evt2` (Prophesee EVT 2.0) or `dvs1` (legacy) |
| `--noise-filter` | 3 | Gaussian blur kernel size |
| `--max-events` | 1000 | Max events/frame for UART send path (auto-clamped to link budget) |
| `--subsample-mode` | spatial | UART downsampling mode: `spatial` or `uniform` |
| `--loop` | False | Loop video file playback |
| `--no-noise` | False | Disable background noise model |
| `--aspect-mode` | crop | Input geometry mode: `crop` (aspect-preserving) or `stretch` |
| `--roi-scale` | 1.0 | Center ROI scale (0.2..1.0), lower reduces background |
| `--flip-x` | False | Horizontally flip input before event generation |
| `--leak-rate` | 0.001 | Reference leak rate per second |
| `--shot-noise` | 0.0001 | Shot noise probability per pixel per frame |

### Event Player

**Replay saved events:**
```bash
python dvs_event_player.py recording.bin --port /dev/ttyUSB0 --preview
```

**Adjust playback speed:**
```bash
# 2x speed
python dvs_event_player.py recording.bin --port /dev/ttyUSB0 --speed 2.0

# Slow motion
python dvs_event_player.py recording.bin --port /dev/ttyUSB0 --speed 0.5
```

**Loop playback:**
```bash
python dvs_event_player.py recording.bin --port /dev/ttyUSB0 --loop
```

## Keyboard Controls (Preview Mode)

| Key | Action |
|-----|--------|
| `q` | Quit |
| `+` / `-` | Increase / decrease contrast threshold |
| `r` | Reset statistics |
| `Space` | Pause / resume |
| `n` | Toggle noise model on/off |
| `1` | Trigger UP gesture (simulation mode) |
| `2` | Trigger DOWN gesture (simulation mode) |
| `3` | Trigger LEFT gesture (simulation mode) |
| `4` | Trigger RIGHT gesture (simulation mode) |

## UART Protocol

When `--port` is used, the emulator streams **EVT 2.0 words** over UART to the FPGA (MSB-first per 32-bit word), matching the input expected by `evt2_decoder.sv` in both architectures.

CD event packet format:

| Bits | Field | Description |
|------|-------|-------------|
| `[31:28]` | `type` | `0x0`=CD_OFF, `0x1`=CD_ON |
| `[27:22]` | `ts_lsb` | 6-bit timestamp LSB (microseconds) |
| `[21:11]` | `x` | 11-bit X coordinate |
| `[10:0]` | `y` | 11-bit Y coordinate |

TIME_HIGH packet format:

| Bits | Field | Description |
|------|-------|-------------|
| `[31:28]` | `type` | `0x8` = TIME_HIGH |
| `[27:0]` | `time_high` | Upper 28 bits of timestamp |

The emulator emits TIME_HIGH whenever upper timestamp bits change, so the FPGA receives a faithful EVT2 stream.

Gesture labels shown by the emulator UI/console are based on **actual UART output from the FPGA**:
- `voxel_bin`: binary gesture packets (`0xA0|gesture`, confidence)
- `gradient_map`: ASCII labels (`UP`, `DOWN`, `LEFT`, `RIGHT`)

## Understanding DVS Events

### ON vs OFF Events

- **ON events** (polarity=1, shown in **blue**): Pixel brightness increased
- **OFF events** (polarity=0, shown in **red**): Pixel brightness decreased

### What Generates Events

| Motion | Leading Edge | Trailing Edge |
|--------|--------------|---------------|
| Bright object on dark background | ON events | OFF events |
| Dark object on bright background | OFF events | ON events |
| Moving texture | Mixed ON/OFF | Mixed ON/OFF |

### Contrast Threshold Explained

The contrast threshold θ determines sensitivity. Real DVS fires when:

$$\log(I_{current}) - \log(I_{reference}) > \theta$$

| Threshold | % Intensity Change | Use Case |
|-----------|-------------------|----------|
| 0.10 | ~10% | High sensitivity, more noise |
| 0.15 | ~15% | Default, balanced |
| 0.20 | ~22% | Less sensitive |
| 0.30 | ~35% | Only strong edges |

## Tips for Best Results

1. **Lighting**: Consistent, even lighting produces cleaner events. Avoid direct sunlight or flickering lights (60Hz flicker can cause noise).

2. **Threshold tuning**: 
   - Start with default contrast (0.15)
   - Lower values = more events, may include noise
   - Higher values = fewer events, misses subtle motion

3. **Background**: A plain, non-moving background reduces noise events.

4. **Camera placement**: Mount camera steadily to avoid whole-frame events from camera shake.

5. **Gestures**: For gesture detection:
   - Move hand across camera view in clear UP/DOWN/LEFT/RIGHT motions
   - Speed should be moderate (not too fast or slow)
   - Cover ~1/3 to 1/2 of the frame with your gesture

6. **Noise model**: Enable noise (`--no-noise` to disable) for realistic testing, but disable it if you need clean ground-truth events.

## Event File Format

### EVT 2.0 (default, `--format evt2`)

The default output format matches the Prophesee EVT 2.0 protocol used by the GenX320 sensor and decoded by `evt2_decoder.sv`. Each packet is a **32-bit little-endian word**:

| Bits | Field | Description |
|------|-------|-------------|
| `[31:28]` | `type` | `0x0`=CD_OFF, `0x1`=CD_ON, `0x8`=TIME_HIGH |
| `[27:22]` | `ts_lsb` | 6-bit timestamp LSB (microseconds) |
| `[21:11]` | `x` | 11-bit X coordinate (0–319) |
| `[10:0]` | `y` | 11-bit Y coordinate (0–319) |

TIME_HIGH packets carry the upper 28 bits of the microsecond timestamp in `[27:0]` and are emitted whenever the upper bits change. The full timestamp is reconstructed as `{time_high[27:0], ts_lsb[5:0]}` (34-bit microseconds).

No file header — the stream begins directly with EVT 2.0 words, identical to a raw capture from the sensor.

### DVS1 (legacy, `--format dvs1`)

```
Header: "DVS1" (4 bytes)
Events: [x(2), y(2), polarity(1), timestamp_us(4)] = 9 bytes each
```

All values are little-endian. Used by `dvs_event_player.py`.

## Troubleshooting

**No camera found:**
- Check camera is connected and not in use by another application
- Try different camera ID: `--camera 1`
- Use simulation mode instead: `--simulate`

**No events generated:**
- Lower contrast threshold: `--contrast 0.08`
- Add more motion in front of camera
- Check camera is not covered or in complete darkness
- Try legacy mode: `--legacy-mode --threshold 10`

**Too many events (noisy):**
- Increase contrast threshold: `--contrast 0.25`
- Enable/increase noise filter: `--noise-filter 5`
- Disable noise model: `--no-noise`
- Improve lighting conditions

**UART connection issues:**
- Verify correct port name
- On Windows, check Device Manager for the active COM port (e.g., `COM3`)
- Check baud rate matches FPGA (115200)
- Ensure FPGA is programmed and running
- Test with echo command: send `0xFF`, should receive `0x55`

**Preview window crashes / `cvShowImage` not implemented:**
- You are using a headless OpenCV build (`opencv-python-headless`)
- Install GUI-enabled OpenCV: `pip install --upgrade opencv-python`
- If needed, remove headless build: `pip uninstall -y opencv-python-headless`
- You can still run without preview by omitting `--preview`

**Events look different from real DVS:**
- Use realistic mode (not `--legacy-mode`)
- Enable noise model (remove `--no-noise`)
- Adjust contrast threshold to match your DVS sensor's settings

## Limitations vs Real DVS

| Aspect | This Emulator | Real DVS |
|--------|---------------|----------|
| Temporal resolution | Frame-based (~33ms) | Microsecond-level |
| Event timing | Same timestamp per frame | True asynchronous |
| Dynamic range | 8-bit camera | 120+ dB |
| Latency | ~50-100ms | <1ms |
| Power | Full camera + PC | <10mW |

For highest accuracy testing, consider using real DVS recordings from public datasets like [DVS-Gesture](https://research.ibm.com/interactive/dvsgesture/) or [DDD17](https://docs.prophesee.ai/).
