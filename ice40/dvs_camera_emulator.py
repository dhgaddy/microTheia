#!/usr/bin/env python3
"""DVS camera emulator: generates DVS-like events from a webcam, video file, or synthetic gestures.

Simulates a Prophesee GenX320 dynamic vision sensor.
Events are encoded in EVT 2.0 format (32-bit big-endian words over UART) to match
the voxel_bin_top FPGA design.
"""

import argparse
import time
import sys
import struct
from typing import Optional, Tuple, List
import threading
import queue
import math

import numpy as np

try:
    import cv2
except ImportError:
    print("ERROR: OpenCV is required. Install with: pip install opencv-python")
    sys.exit(1)

try:
    import serial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False
    print("WARNING: pyserial not installed. UART output disabled.")
    print("         Install with: pip install pyserial")


DVS_RESOLUTION = 320           # GenX320: 320×320 pixels
DEFAULT_CONTRAST_THRESHOLD = 0.15  # ~15% log-intensity change per event
DEFAULT_REFRACTORY_US = 200   # GenX320 min refractory ~200 µs
DEFAULT_FPS = 30
DEFAULT_LEAK_RATE = 0.05   # fraction of (log_frame - reference) to close per second
DEFAULT_SHOT_NOISE_RATE = 0.0001
DEFAULT_HOT_PIXEL_RATE = 0.00005


def get_available_serial_ports() -> List[str]:
    if not HAS_SERIAL:
        return []
    try:
        from serial.tools import list_ports
        ports = []
        for port in list_ports.comports():
            description = (port.description or "").strip()
            if description and description.lower() != "n/a":
                ports.append(f"{port.device} ({description})")
            else:
                ports.append(port.device)
        return ports
    except Exception:
        return []


def opencv_gui_available() -> bool:
    try:
        test_window = "__dvs_gui_test__"
        cv2.namedWindow(test_window, cv2.WINDOW_NORMAL)
        cv2.destroyWindow(test_window)
        return True
    except cv2.error:
        return False


def safe_destroy_all_windows():
    try:
        cv2.destroyAllWindows()
    except cv2.error:
        pass


def estimate_uart_event_budget_per_frame(baud_rate: int, fps: int) -> int:
    """Estimate sustainable event budget/frame for EVT2 over 8N1 UART.

    UART carries roughly baud/10 payload bytes per second. EVT2 sends 4-byte words;
    each event uses one CD word plus occasional TIME_HIGH words. A 0.90 margin is
    used to keep queue occupancy stable under bursty camera traffic.
    """
    if baud_rate <= 0 or fps <= 0:
        return 1
    bytes_per_second = baud_rate / 10.0
    words_per_second = bytes_per_second / 4.0
    frame_budget = words_per_second / float(fps)
    return max(1, int(math.floor(frame_budget * 0.90)))


def uniform_subsample_events(events: List['DVSEvent'], limit: int) -> List['DVSEvent']:
    """Keep a spatially/time-distributed subset without top-left scan bias."""
    if limit <= 0:
        return []
    event_count = len(events)
    if event_count <= limit:
        return events
    idx = np.linspace(0, event_count - 1, num=limit, dtype=np.int64)
    return [events[int(i)] for i in idx]


def spatial_subsample_events(
    events: List['DVSEvent'],
    limit: int,
    sensor_resolution: int = DVS_RESOLUTION,
    grid_size: int = 16
) -> List['DVSEvent']:
    """Round-robin sample across spatial-cell buckets.

    This preserves directional structure when heavy event bursts are throttled
    to UART link capacity.
    """
    if limit <= 0:
        return []
    if len(events) <= limit:
        return events

    cell_size = max(1, sensor_resolution // grid_size)
    buckets = {}
    for ev in events:
        cx = min(grid_size - 1, max(0, ev.x // cell_size))
        cy = min(grid_size - 1, max(0, ev.y // cell_size))
        key = (cy << 4) | cx
        if key not in buckets:
            buckets[key] = []
        buckets[key].append(ev)

    active = sorted(buckets.keys())
    read_idx = {k: 0 for k in active}
    selected: List['DVSEvent'] = []

    while active and len(selected) < limit:
        next_active = []
        for key in active:
            idx = read_idx[key]
            bucket = buckets[key]
            if idx < len(bucket):
                selected.append(bucket[idx])
                read_idx[key] = idx + 1
                if read_idx[key] < len(bucket):
                    next_active.append(key)
                if len(selected) >= limit:
                    break
        active = next_active

    return selected


class GestureSimulator:
    """Generates synthetic frames with moving objects for testing without a camera."""
    
    def __init__(self, resolution: int = DVS_RESOLUTION, fps: int = DEFAULT_FPS):
        self.resolution = resolution
        self.fps = fps
        self.frame_count = 0
        self.gesture_queue = []
        self.current_gesture = None
        self.gesture_frame = 0
        self.gesture_duration = 30
        self.add_gesture('right')
        self.add_gesture('left')
        self.add_gesture('down')
        self.add_gesture('up')
    
    def add_gesture(self, direction: str):
        if direction is not None:
            self.gesture_queue.append(direction)
    
    def get_frame(self) -> np.ndarray:
        frame = np.ones((self.resolution, self.resolution, 3), dtype=np.uint8) * 40
        while self.gesture_queue and self.gesture_queue[0] is None:
            self.gesture_queue.pop(0)
        if self.current_gesture is None and self.gesture_queue:
            self.current_gesture = self.gesture_queue.pop(0)
            self.gesture_frame = 0
            if self.current_gesture:
                print(f"Simulating gesture: {self.current_gesture.upper()}")
        
        if self.current_gesture:
            progress = self.gesture_frame / self.gesture_duration
            center = self.resolution // 2
            obj_size = self.resolution // 4
            # NOTE: Training camera was mounted with both axes inverted vs grid coords.
            # Physical RIGHT -> grid x sweeps HIGH->LOW, so object x moves (res->0) in screen space.
            # Physical LEFT  -> grid x sweeps LOW->HIGH, so object x moves (0->res) in screen space.
            # Physical DOWN  -> grid y sweeps HIGH->LOW, so object y moves (res->0) in screen space.
            # Physical UP    -> grid y sweeps LOW->HIGH, so object y moves (0->res) in screen space.
            if self.current_gesture == 'right':
                x = int((1 - progress) * (self.resolution - obj_size))
                y = center - obj_size // 2
            elif self.current_gesture == 'left':
                x = int(progress * (self.resolution - obj_size))
                y = center - obj_size // 2
            elif self.current_gesture == 'down':
                x = center - obj_size // 2
                y = int((1 - progress) * (self.resolution - obj_size))
            elif self.current_gesture == 'up':
                x = center - obj_size // 2
                y = int(progress * (self.resolution - obj_size))
            else:
                x, y = center, center
            cv2.rectangle(frame, (x, y), (x + obj_size, y + obj_size), (200, 200, 200), -1)
            cv2.circle(frame, (x + obj_size//3, y + obj_size//3), obj_size//6, (180, 180, 180), -1)
            cv2.circle(frame, (x + 2*obj_size//3, y + obj_size//3), obj_size//6, (180, 180, 180), -1)
            self.gesture_frame += 1
            if self.gesture_frame >= self.gesture_duration:
                self.current_gesture = None
        if self.current_gesture is None and not self.gesture_queue:
            self.add_gesture('right')
            self.add_gesture('left')
            self.add_gesture('down')
            self.add_gesture('up')
        
        self.frame_count += 1
        return frame
    
    def is_running(self) -> bool:
        return True


class DVSEvent:
    def __init__(self, x: int, y: int, polarity: bool, timestamp_us: int):
        self.x = x
        self.y = y
        self.polarity = polarity
        self.timestamp_us = timestamp_us
    
    def __repr__(self):
        pol_str = "ON" if self.polarity else "OFF"
        return f"DVSEvent(x={self.x}, y={self.y}, pol={pol_str}, ts={self.timestamp_us}μs)"


class DVSCameraEmulator:
    """Emulates a DVS sensor using a webcam with per-pixel log-intensity reference tracking."""
    
    def __init__(
        self,
        camera_id: int = 0,
        output_resolution: int = DVS_RESOLUTION,
        contrast_threshold: float = DEFAULT_CONTRAST_THRESHOLD,
        refractory_period_us: int = DEFAULT_REFRACTORY_US,
        fps: int = DEFAULT_FPS,
        noise_filter_size: int = 3,
        enable_noise_model: bool = True,
        leak_rate: float = DEFAULT_LEAK_RATE,
        shot_noise_rate: float = DEFAULT_SHOT_NOISE_RATE,
        hot_pixel_rate: float = DEFAULT_HOT_PIXEL_RATE,
        aspect_mode: str = 'stretch',
        roi_scale: float = 1.0,
        flip_x: bool = False,
        flip_y: bool = False,
        rotate_cw: bool = False,
    ):
        self.camera_id = camera_id
        self.output_resolution = output_resolution
        self.contrast_threshold = contrast_threshold
        self.refractory_period_us = refractory_period_us
        self.target_fps = fps
        self.noise_filter_size = noise_filter_size
        self.enable_noise_model = enable_noise_model
        self.leak_rate = leak_rate
        self.shot_noise_rate = shot_noise_rate
        self.hot_pixel_rate = hot_pixel_rate
        self.aspect_mode = aspect_mode
        self.roi_scale = float(max(0.2, min(1.0, roi_scale)))
        self.flip_x = flip_x
        self.flip_y = flip_y
        self.rotate_cw = rotate_cw
        self.cap: Optional[cv2.VideoCapture] = None
        self.reference_log_intensity: Optional[np.ndarray] = None
        self.last_event_time: Optional[np.ndarray] = None
        self.hot_pixel_mask: Optional[np.ndarray] = None
        self.start_time_us = 0
        self.last_frame_time_us = 0
        self.frame_count = 0
        self.total_events = 0
        self.on_events = 0
        self.off_events = 0
        self.noise_events = 0
    
    def open_camera(self) -> bool:
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            print(f"ERROR: Could not open camera {self.camera_id}")
            return False
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, self.target_fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        print(f"Camera opened: {actual_width}x{actual_height} @ {actual_fps:.1f} FPS")
        print(f"Output resolution: {self.output_resolution}x{self.output_resolution}")
        print(f"Contrast threshold: {self.contrast_threshold:.2f} ({self.contrast_threshold*100:.0f}% intensity change)")
        print(f"Refractory period: {self.refractory_period_us}μs")
        print(f"Noise model: {'ENABLED' if self.enable_noise_model else 'DISABLED'}")
        self._initialize_pixel_state()
        return True
    
    def _initialize_pixel_state(self):
        shape = (self.output_resolution, self.output_resolution)
        self.reference_log_intensity = None
        self.last_event_time = np.zeros(shape, dtype=np.int64)
        if self.enable_noise_model:
            num_hot_pixels = int(self.output_resolution * self.output_resolution * self.hot_pixel_rate)
            self.hot_pixel_mask = np.zeros(shape, dtype=bool)
            hot_indices = np.random.choice(
                self.output_resolution * self.output_resolution,
                size=num_hot_pixels,
                replace=False
            )
            self.hot_pixel_mask.flat[hot_indices] = True
        self.start_time_us = int(time.time() * 1_000_000)
        self.last_frame_time_us = self.start_time_us
    
    def close_camera(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None
    
    def _preprocess_frame(self, frame: np.ndarray) -> np.ndarray:
        if len(frame.shape) == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame

        if self.flip_x and self.flip_y:
            gray = cv2.flip(gray, -1)  # flip both axes
        elif self.flip_x:
            gray = cv2.flip(gray, 1)   # horizontal flip only
        elif self.flip_y:
            gray = cv2.flip(gray, 0)   # vertical flip only

        if self.rotate_cw:
            gray = cv2.rotate(gray, cv2.ROTATE_90_CLOCKWISE)  # equiv to --swap-xy --flip-x

        if self.roi_scale < 0.999:
            h, w = gray.shape
            crop_h = max(1, int(h * self.roi_scale))
            crop_w = max(1, int(w * self.roi_scale))
            y0 = max(0, (h - crop_h) // 2)
            x0 = max(0, (w - crop_w) // 2)
            gray = gray[y0:y0 + crop_h, x0:x0 + crop_w]

        if self.aspect_mode == 'crop':
            h, w = gray.shape
            side = min(h, w)
            y0 = (h - side) // 2
            x0 = (w - side) // 2
            square = gray[y0:y0 + side, x0:x0 + side]
            resized = cv2.resize(
                square,
                (self.output_resolution, self.output_resolution),
                interpolation=cv2.INTER_AREA
            )
        else:
            resized = cv2.resize(
                gray,
                (self.output_resolution, self.output_resolution),
                interpolation=cv2.INTER_AREA
            )
        if self.noise_filter_size > 1:
            resized = cv2.GaussianBlur(
                resized, 
                (self.noise_filter_size, self.noise_filter_size), 
                0
            )
        log_frame = np.log(resized.astype(np.float32) + 1.0)
        return log_frame
    
    def _apply_reference_leak(self, log_frame: np.ndarray, dt_seconds: float):
        """Leak the per-pixel reference toward the current log frame value.

        Real DVS pixels have a photoreceptor capacitor that continuously charges
        toward the current light level. Modelling the leak this way means idle
        pixels (no motion) smoothly track scene illumination without accumulating
        a large diff that fires spurious events when motion stops.
        """
        if self.reference_log_intensity is None:
            return
        leak_amount = min(1.0, self.leak_rate * dt_seconds)
        # Leak toward current scene value, not the global mean.
        self.reference_log_intensity += leak_amount * (log_frame - self.reference_log_intensity)
    
    def _generate_noise_events(self, current_time_us: int) -> List['DVSEvent']:
        if not self.enable_noise_model:
            return []
        noise_events = []
        noise_mask = np.random.random((self.output_resolution, self.output_resolution)) < self.shot_noise_rate
        hot_noise_mask = np.random.random((self.output_resolution, self.output_resolution)) < (self.shot_noise_rate * 10)
        noise_mask = noise_mask | (self.hot_pixel_mask & hot_noise_mask)
        time_since_last = current_time_us - self.last_event_time
        refractory_mask = time_since_last >= self.refractory_period_us
        noise_mask = noise_mask & refractory_mask
        noise_coords = np.argwhere(noise_mask)
        for y, x in noise_coords:
            polarity = np.random.random() > 0.5
            event = DVSEvent(x=int(x), y=int(y), polarity=polarity, timestamp_us=current_time_us)
            noise_events.append(event)
            self.last_event_time[y, x] = current_time_us
            self.noise_events += 1
        return noise_events
    
    def process_frame(self, frame: np.ndarray) -> List[DVSEvent]:
        current_time_us = int(time.time() * 1_000_000) - self.start_time_us
        dt_seconds = (current_time_us - self.last_frame_time_us) / 1_000_000.0
        log_frame = self._preprocess_frame(frame)
        if self.reference_log_intensity is None:
            self.reference_log_intensity = log_frame.copy()
            self.last_frame_time_us = current_time_us
            return []
        self._apply_reference_leak(log_frame, dt_seconds)
        events = []
        diff = log_frame - self.reference_log_intensity
        on_mask = diff > self.contrast_threshold
        off_mask = diff < -self.contrast_threshold
        time_since_last = current_time_us - self.last_event_time
        refractory_mask = time_since_last >= self.refractory_period_us
        on_mask = on_mask & refractory_mask
        off_mask = off_mask & refractory_mask
        # Pixels that fired snap their reference to current log intensity immediately.
        on_coords = np.argwhere(on_mask)
        for y, x in on_coords:
            event = DVSEvent(x=int(x), y=int(y), polarity=True, timestamp_us=current_time_us)
            events.append(event)
            self.last_event_time[y, x] = current_time_us
            self.reference_log_intensity[y, x] = log_frame[y, x]
            self.on_events += 1
        off_coords = np.argwhere(off_mask)
        for y, x in off_coords:
            event = DVSEvent(x=int(x), y=int(y), polarity=False, timestamp_us=current_time_us)
            events.append(event)
            self.last_event_time[y, x] = current_time_us
            self.reference_log_intensity[y, x] = log_frame[y, x]
            self.off_events += 1
        noise_events = self._generate_noise_events(current_time_us)
        events.extend(noise_events)
        self.total_events += len(events)
        self.frame_count += 1
        self.last_frame_time_us = current_time_us
        return events
    
    def capture_and_process(self) -> Tuple[Optional[np.ndarray], List[DVSEvent]]:
        if self.cap is None or not self.cap.isOpened():
            return None, []
        ret, frame = self.cap.read()
        if not ret:
            return None, []
        events = self.process_frame(frame)
        return frame, events
    
    def get_stats(self) -> dict:
        return {
            'total_events': self.total_events,
            'on_events': self.on_events,
            'off_events': self.off_events,
            'noise_events': self.noise_events,
            'frame_count': self.frame_count,
            'events_per_frame': self.total_events / max(1, self.frame_count)
        }


def create_event_visualization(
    events: List[DVSEvent], 
    resolution: int = DVS_RESOLUTION,
    accumulation_time_ms: int = 33
) -> np.ndarray:
    """ON events -> blue, OFF events -> red."""
    vis = np.zeros((resolution, resolution, 3), dtype=np.uint8)
    for event in events:
        x, y = event.x, event.y
        if 0 <= x < resolution and 0 <= y < resolution:
            if event.polarity:
                vis[y, x, 0] = 255
            else:
                vis[y, x, 2] = 255
    return vis


def create_combined_preview(
    original_frame: np.ndarray,
    events: List[DVSEvent],
    resolution: int = DVS_RESOLUTION,
    stats: dict = None,
    recent_gestures: List[str] = None
) -> np.ndarray:
    orig_resized = cv2.resize(original_frame, (resolution, resolution), interpolation=cv2.INTER_AREA)
    event_vis = create_event_visualization(events, resolution)
    combined = np.hstack([orig_resized, event_vis])
    y_offset = 20
    if stats:
        text = f"Events: {stats['total_events']} | ON: {stats['on_events']} | OFF: {stats['off_events']}"
        cv2.putText(combined, text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        y_offset += 20
        fps_text = f"Frames: {stats['frame_count']} | Events/frame: {stats['events_per_frame']:.1f}"
        cv2.putText(combined, fps_text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        y_offset += 20
    if recent_gestures:
        y_offset += 10
        cv2.putText(combined, "FPGA Gestures (UART RX):", (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1, cv2.LINE_AA)
        y_offset += 20
        for i, gesture in enumerate(recent_gestures[-3:]):
            cv2.putText(combined, f"  {gesture}", (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
            y_offset += 20
    return combined


class UARTOutputHandler:
    """Sends EVT 2.0 events to the voxel_bin_top FPGA over UART and receives gesture packets."""

    def __init__(self, port: str, baud_rate: int = 115200):
        self.port = port
        self.baud_rate = baud_rate
        self.serial: Optional[serial.Serial] = None
        self.event_queue: queue.Queue = queue.Queue(maxsize=10000)
        self.running = False
        self.tx_thread: Optional[threading.Thread] = None
        self.rx_thread: Optional[threading.Thread] = None
        self.events_sent = 0
        self.evt2_words_sent = 0
        self.queue_events_dropped = 0
        self.queue_high_watermark = 0
        self.gestures_received = []
        self.rx_buffer = bytearray()
        self._last_time_high = -1
        self.lock = threading.Lock()

    def _encode_evt2_words(self, event: DVSEvent) -> List[int]:
        words: List[int] = []
        ts = int(event.timestamp_us) & ((1 << 34) - 1)
        time_high = (ts >> 6) & 0x0FFFFFFF
        ts_lsb = ts & 0x3F

        if time_high != self._last_time_high:
            words.append((0x8 << 28) | time_high)
            self._last_time_high = time_high

        evt_type = 0x1 if event.polarity else 0x0
        x = event.x & 0x7FF
        y = event.y & 0x7FF
        words.append((evt_type << 28) | (ts_lsb << 22) | (x << 11) | y)
        return words

    @staticmethod
    def _word_to_uart_bytes(word: int) -> bytes:
        return bytes([
            (word >> 24) & 0xFF,
            (word >> 16) & 0xFF,
            (word >> 8) & 0xFF,
            word & 0xFF,
        ])
    
    def open(self) -> bool:
        if not HAS_SERIAL:
            print("ERROR: pyserial not installed")
            return False
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            print(f"UART opened: {self.port} @ {self.baud_rate} baud")
            return True
        except serial.SerialException as e:
            print(f"ERROR: Could not open {self.port}: {e}")
            error_text = str(e).lower()
            if "access is denied" in error_text or "permission" in error_text:
                print("TIP: Port is busy. Close any serial monitor/terminal using this COM port and retry.")
            ports = get_available_serial_ports()
            if ports:
                print(f"Available serial ports: {', '.join(ports)}")
            else:
                print("No serial ports detected. Check cable/driver/device power.")
            if sys.platform == 'win32':
                print("TIP: On Windows, use Device Manager to confirm the COM number (e.g., COM3).")
            return False
    
    def close(self):
        self.running = False
        if self.tx_thread:
            self.tx_thread.join(timeout=1.0)
        if self.rx_thread:
            self.rx_thread.join(timeout=1.0)
        if self.serial:
            self.serial.close()
            self.serial = None
    
    def get_recent_gestures(self, max_count: int = 5) -> List[str]:
        with self.lock:
            return [name for name, _ in self.gestures_received[-max_count:]]
    
    def start_tx_thread(self):
        self.running = True
        self.tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self.tx_thread.start()
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()
    
    def _tx_loop(self):
        while self.running:
            try:
                event = self.event_queue.get(timeout=0.1)
                if self.serial and self.serial.is_open:
                    evt2_words = self._encode_evt2_words(event)
                    for word in evt2_words:
                        self.serial.write(self._word_to_uart_bytes(word))
                        self.evt2_words_sent += 1
                    self.events_sent += 1
            except queue.Empty:
                continue
            except Exception as e:
                print(f"TX Error: {e}")
    
    def _rx_loop(self):
        # FPGA gesture encoding: 0=Down, 1=Left, 2=Right, 3=Up (matches voxel_bin_top TX)
        gesture_names = ['Down', 'Left', 'Right', 'Up']
        while self.running:
            try:
                if self.serial and self.serial.is_open and self.serial.in_waiting > 0:
                    data = self.serial.read(self.serial.in_waiting)
                    self.rx_buffer.extend(data)
                    while len(self.rx_buffer) > 0:
                        byte0 = self.rx_buffer[0]

                        # Binary voxel_bin gesture packet: [0xA0|gesture, confidence]
                        if (byte0 & 0xF0) == 0xA0:
                            if len(self.rx_buffer) < 2:
                                break
                            pkt = self.rx_buffer[0]
                            conf_byte = self.rx_buffer[1]
                            del self.rx_buffer[:2]

                            gesture_idx = pkt & 0x03
                            gesture_str = gesture_names[gesture_idx]
                            confidence = (conf_byte >> 4) & 0x0F
                            with self.lock:
                                self.gestures_received.append((gesture_str, time.time()))
                            print(f"\n*** GESTURE DETECTED: {gesture_str} (conf={confidence}) ***")
                            continue

                        # Unexpected byte — discard
                        del self.rx_buffer[0]
                else:
                    time.sleep(0.01)
            except Exception as e:
                print(f"RX Error: {e}")
                time.sleep(0.1)
    
    def send_event(self, event: DVSEvent):
        while True:
            try:
                self.event_queue.put_nowait(event)
                try:
                    self.queue_high_watermark = max(self.queue_high_watermark, self.event_queue.qsize())
                except Exception:
                    pass
                return True
            except queue.Full:
                # Drop oldest to prioritize freshest events for lower end-to-end latency.
                try:
                    self.event_queue.get_nowait()
                    self.queue_events_dropped += 1
                except queue.Empty:
                    self.queue_events_dropped += 1
                    return False
    
    def send_events(self, events: List[DVSEvent]) -> int:
        enqueued = 0
        for event in events:
            if self.send_event(event):
                enqueued += 1
        return enqueued
    
    def test_connection(self) -> bool:
        """Send echo command (0xFF) and verify voxel_bin_top responds with 0x55."""
        if not self.serial or not self.serial.is_open:
            return False
        try:
            self.serial.reset_input_buffer()
            self.serial.write(bytes([0xFF]))
            time.sleep(0.1)
            if self.serial.in_waiting > 0:
                response = self.serial.read(1)
                if response[0] == 0x55:
                    print("FPGA connection verified (echo 0xFF -> 0x55)")
                    return True
            print("WARNING: echo test did not return 0x55 — check bitstream and baud rate")
            return False
        except Exception as e:
            print(f"Connection test failed: {e}")
            return False


class EVT2FileOutputHandler:
    """Saves DVS events to a binary file in Prophesee EVT 2.0 format.

    Each packet is a 32-bit little-endian word:
      [31:28] type  — 0x0=CD_OFF, 0x1=CD_ON, 0x8=TIME_HIGH
      [27:22] ts_lsb — 6-bit timestamp LSB (microseconds)
      [21:11] x     — 11-bit X coordinate (0–319)
      [10:0]  y     — 11-bit Y coordinate (0–319)

    TIME_HIGH packets carry the upper 28 bits of the timestamp in [27:0]
    and are emitted whenever the upper bits change.
    """

    EVT_CD_OFF    = 0x0
    EVT_CD_ON     = 0x1
    EVT_TIME_HIGH = 0x8

    def __init__(self, filename: str):
        self.filename = filename
        self.file = None
        self.events_written = 0
        self._last_time_high: int = -1

    def open(self):
        self.file = open(self.filename, 'wb')
        print(f"Saving events to: {self.filename} (EVT 2.0 format)")

    def close(self):
        if self.file:
            self.file.close()
            print(f"Saved {self.events_written} events to {self.filename}")

    def _write_word(self, word: int):
        self.file.write(struct.pack('<I', word))

    def write_event(self, event: DVSEvent):
        if not self.file:
            return
        ts = event.timestamp_us
        time_high = (ts >> 6) & 0x0FFFFFFF
        ts_lsb    = ts & 0x3F

        if time_high != self._last_time_high:
            th_word = (self.EVT_TIME_HIGH << 28) | time_high
            self._write_word(th_word)
            self._last_time_high = time_high

        pkt_type = self.EVT_CD_ON if event.polarity else self.EVT_CD_OFF
        x = event.x & 0x7FF
        y = event.y & 0x7FF
        word = (pkt_type << 28) | (ts_lsb << 22) | (x << 11) | y
        self._write_word(word)
        self.events_written += 1

    def write_events(self, events: List[DVSEvent]):
        for event in events:
            self.write_event(event)


def main():
    parser = argparse.ArgumentParser(description='DVS Camera Emulator')
    
    parser.add_argument('--camera', type=int, default=0,
                        help='Camera device ID (default: 0)')
    parser.add_argument('--video', type=str, default=None,
                        help='Video file to use as input instead of camera')
    parser.add_argument('--simulate', action='store_true',
                        help='Simulate gestures without camera (for testing)')
    parser.add_argument('--port', type=str, default=None,
                        help='Serial port for UART output (e.g., /dev/ttyUSB0, COM3)')
    parser.add_argument('--baud', type=int, default=115200,
                        help='UART baud rate (default: 115200)')
    parser.add_argument('--contrast', type=float, default=DEFAULT_CONTRAST_THRESHOLD,
                        help=f'Contrast threshold: log-intensity change per event (default: {DEFAULT_CONTRAST_THRESHOLD}, ~15%% change)')
    parser.add_argument('--refractory', type=int, default=DEFAULT_REFRACTORY_US,
                        help=f'Refractory period in μs (default: {DEFAULT_REFRACTORY_US})')
    parser.add_argument('--resolution', type=int, default=DVS_RESOLUTION,
                        help=f'Output resolution (default: {DVS_RESOLUTION})')
    parser.add_argument('--fps', type=int, default=DEFAULT_FPS,
                        help=f'Camera FPS (default: {DEFAULT_FPS})')
    parser.add_argument('--preview', action='store_true',
                        help='Show preview window')
    parser.add_argument('--save', type=str, default=None,
                        help='Save events to EVT 2.0 binary file (.raw)')
    parser.add_argument('--noise-filter', type=int, default=3,
                        help='Gaussian blur kernel size for noise filtering (default: 3, 0=disabled)')
    parser.add_argument('--max-events', type=int, default=1000,
                        help='Maximum events/frame for UART send path (default: 1000, auto-clamped to link budget)')
    parser.add_argument('--subsample-mode', type=str, default='spatial', choices=['spatial', 'uniform'],
                        help='UART downsampling strategy when events exceed per-frame budget (default: spatial)')
    parser.add_argument('--loop', action='store_true',
                        help='Loop video file playback')
    parser.add_argument('--aspect-mode', type=str, default='crop', choices=['crop', 'stretch'],
                        help='Input resize mode before DVS conversion (default: crop to preserve motion geometry)')
    parser.add_argument('--roi-scale', type=float, default=1.0,
                        help='Center ROI scale in (0.2..1.0], lower removes peripheral background (default: 1.0)')
    parser.add_argument('--flip-x', action='store_true',
                        help='Horizontally flip input before DVS conversion')
    parser.add_argument('--flip-y', action='store_true',
                        help='Vertically flip input before DVS conversion')
    parser.add_argument('--flip-both', action='store_true',
                        help='Flip both axes (equivalent to --flip-x --flip-y); use when camera is mounted upside-down or both axes are inverted')
    parser.add_argument('--rotate-cw', action='store_true',
                        help='Rotate image 90° clockwise before DVS conversion (equivalent to --swap-xy --flip-x at EVT2 level; use for this camera orientation)')
    parser.add_argument('--no-noise', action='store_true',
                        help='Disable background noise model')
    parser.add_argument('--leak-rate', type=float, default=DEFAULT_LEAK_RATE,
                        help=f'Reference leak rate per second (default: {DEFAULT_LEAK_RATE})')
    parser.add_argument('--shot-noise', type=float, default=DEFAULT_SHOT_NOISE_RATE,
                        help=f'Shot noise probability per pixel per frame (default: {DEFAULT_SHOT_NOISE_RATE})')
    
    args = parser.parse_args()
    preview_enabled = args.preview
    if preview_enabled and not opencv_gui_available():
        print("WARNING: OpenCV GUI backend is not available. Running without preview window.")
        print("         Install GUI-enabled OpenCV: pip install --upgrade opencv-python")
        print("         If needed, remove headless build: pip uninstall -y opencv-python-headless")
        preview_enabled = False

    use_simulator = args.simulate
    use_video = args.video is not None
    video_cap = None
    simulator = None
    do_flip_x = args.flip_x or args.flip_both
    do_flip_y = args.flip_y or args.flip_both
    do_rotate_cw = args.rotate_cw
    emulator = DVSCameraEmulator(
        camera_id=args.camera,
        output_resolution=args.resolution,
        contrast_threshold=args.contrast,
        refractory_period_us=args.refractory,
        fps=args.fps,
        noise_filter_size=args.noise_filter if args.noise_filter > 0 else 1,
        enable_noise_model=not args.no_noise,
        leak_rate=args.leak_rate,
        shot_noise_rate=args.shot_noise,
        aspect_mode=args.aspect_mode,
        roi_scale=args.roi_scale,
        flip_x=do_flip_x,
        flip_y=do_flip_y,
        rotate_cw=do_rotate_cw,
    )

    print(f"Input geometry: aspect={args.aspect_mode}, roi_scale={max(0.2, min(1.0, args.roi_scale)):.2f}, flip_x={'ON' if do_flip_x else 'OFF'}, flip_y={'ON' if do_flip_y else 'OFF'}, rotate_cw={'ON' if do_rotate_cw else 'OFF'}")
    print(f"UART subsampling: {args.subsample_mode}")
    
    if use_simulator:
        print(f"Mode: SIMULATION ({args.resolution}x{args.resolution})")
        simulator = GestureSimulator(args.resolution, args.fps)
        emulator._initialize_pixel_state()
    elif use_video:
        print(f"Mode: VIDEO FILE ({args.video})")
        video_cap = cv2.VideoCapture(args.video)
        if not video_cap.isOpened():
            print(f"ERROR: Could not open video file: {args.video}")
            sys.exit(1)
        video_fps = video_cap.get(cv2.CAP_PROP_FPS)
        video_frames = int(video_cap.get(cv2.CAP_PROP_FRAME_COUNT))
        video_width = int(video_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        video_height = int(video_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"Video: {video_width}x{video_height} @ {video_fps:.1f} FPS, {video_frames} frames")
        emulator._initialize_pixel_state()
    else:
        print("Mode: CAMERA")
        if not emulator.open_camera():
            print("TIP: Use --simulate or --video FILE if no camera is available.")
            sys.exit(1)
    uart_handler = None
    file_handler = None
    
    if args.port:
        uart_handler = UARTOutputHandler(args.port, args.baud)
        if uart_handler.open():
            if not uart_handler.test_connection():
                print("ERROR: voxel_bin_top echo test failed; check bitstream/baud/port.")
                uart_handler.close()
                uart_handler = None
            else:
                uart_handler.start_tx_thread()
        else:
            uart_handler = None

    uart_budget = args.max_events
    if uart_handler:
        estimated_budget = estimate_uart_event_budget_per_frame(args.baud, args.fps)
        if args.max_events > estimated_budget:
            uart_budget = estimated_budget
            print(f"WARNING: --max-events {args.max_events} exceeds UART capacity at {args.baud} baud / {args.fps} FPS.")
            print(f"         Using UART send budget: {uart_budget} events/frame (estimated sustainable).")
            print("         Increase baud rate or lower FPS for higher live event throughput.")
        else:
            uart_budget = args.max_events
    
    if args.save:
        file_handler = EVT2FileOutputHandler(args.save)
        file_handler.open()
    
    paused = False
    frame_time = 1.0 / args.fps
    uart_events_generated = 0
    uart_events_after_subsample = 0
    uart_events_enqueued = 0
    
    try:
        while True:
            loop_start = time.time()
            
            if not paused:
                frame = None
                if use_simulator:
                    frame = simulator.get_frame()
                elif use_video:
                    ret, frame = video_cap.read()
                    if not ret:
                        if args.loop:
                            video_cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                            ret, frame = video_cap.read()
                        if not ret:
                            print("Video playback complete")
                            break
                else:
                    frame, _ = emulator.capture_and_process()
                    if frame is not None:
                        events = _
                if frame is None and not use_simulator and not use_video:
                    print("Camera error - reconnecting...")
                    time.sleep(1.0)
                    continue
                if use_simulator or use_video:
                    events = emulator.process_frame(frame)
                elif frame is None:
                    continue
                if uart_handler:
                    if args.subsample_mode == 'uniform':
                        tx_events = uniform_subsample_events(events, uart_budget)
                    else:
                        tx_events = spatial_subsample_events(
                            events,
                            uart_budget,
                            sensor_resolution=args.resolution,
                            grid_size=16
                        )
                    uart_events_generated += len(events)
                    uart_events_after_subsample += len(tx_events)
                    uart_events_enqueued += uart_handler.send_events(tx_events)
                if file_handler:
                    file_handler.write_events(events)
                if preview_enabled:
                    stats = emulator.get_stats()
                    recent_gestures = uart_handler.get_recent_gestures(max_count=3) if uart_handler else None
                    preview = create_combined_preview(frame, events, args.resolution, stats, recent_gestures)
                    preview_scaled = cv2.resize(preview, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)
                    try:
                        cv2.imshow('DVS Emulator (Original | Events)', preview_scaled)
                    except cv2.error:
                        print("WARNING: Preview window failed (OpenCV GUI unavailable). Continuing without preview.")
                        preview_enabled = False
                        safe_destroy_all_windows()
            if preview_enabled:
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q'):
                    break
                elif key == ord('+') or key == ord('='):
                    emulator.contrast_threshold += 0.02
                    print(f"Contrast threshold: {emulator.contrast_threshold:.2f} (~{emulator.contrast_threshold*100:.0f}%)")
                elif key == ord('-'):
                    emulator.contrast_threshold = max(0.02, emulator.contrast_threshold - 0.02)
                    print(f"Contrast threshold: {emulator.contrast_threshold:.2f} (~{emulator.contrast_threshold*100:.0f}%)")
                elif key == ord('n'):
                    emulator.enable_noise_model = not emulator.enable_noise_model
                    print(f"Noise model: {'ENABLED' if emulator.enable_noise_model else 'DISABLED'}")
                elif key == ord('r'):
                    emulator.total_events = 0
                    emulator.on_events = 0
                    emulator.off_events = 0
                    emulator.noise_events = 0
                    emulator.frame_count = 0
                    print("Statistics reset")
                elif key == ord(' '):
                    paused = not paused
                    print("Paused" if paused else "Resumed")
                elif use_simulator and key == ord('1'):
                    simulator.add_gesture('up')
                    print("Queued gesture: UP")
                elif use_simulator and key == ord('2'):
                    simulator.add_gesture('down')
                    print("Queued gesture: DOWN")
                elif use_simulator and key == ord('3'):
                    simulator.add_gesture('left')
                    print("Queued gesture: LEFT")
                elif use_simulator and key == ord('4'):
                    simulator.add_gesture('right')
                    print("Queued gesture: RIGHT")
            else:
                time.sleep(0.001)
            elapsed = time.time() - loop_start
            if elapsed < frame_time:
                time.sleep(frame_time - elapsed)
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    
    finally:
        stats = emulator.get_stats()
        print(f"Total Events: {stats['total_events']} | ON: {stats['on_events']} | OFF: {stats['off_events']} | Noise: {stats['noise_events']} | Frames: {stats['frame_count']}")
        if uart_handler:
            print(f"Events Sent: {uart_handler.events_sent}")
            print(f"EVT2 Words Sent: {uart_handler.evt2_words_sent}")
            print(f"UART Events Generated: {uart_events_generated}")
            print(f"UART Events After Subsample: {uart_events_after_subsample}")
            print(f"UART Events Enqueued: {uart_events_enqueued}")
            print(f"UART Queue Drops (oldest evicted): {uart_handler.queue_events_dropped}")
            print(f"UART Queue High Watermark: {uart_handler.queue_high_watermark}")
            with uart_handler.lock:
                if uart_handler.gestures_received:
                    gesture_counts = {}
                    for name, _ in uart_handler.gestures_received:
                        gesture_counts[name] = gesture_counts.get(name, 0) + 1
                    for name, count in gesture_counts.items():
                        print(f"  {name}: {count}")
            uart_handler.close()
        if file_handler:
            file_handler.close()
        if video_cap:
            video_cap.release()
        if not use_simulator and not use_video:
            emulator.close_camera()
        if preview_enabled:
            safe_destroy_all_windows()


if __name__ == '__main__':
    main()
