"""
Autonomous ArUco Navigation

The rover has to visit a fixed sequence of ArUco markers, identified by
(marker ID, marker color), in order — then, once every marker has been
visited, retrace its own GPS track back to the starting point.

High-level flow per loop iteration:
  1. Grab a frame from the Pi camera and look for ArUco markers.
  2. If the marker we're currently looking for is visible, estimate how
     far away it is (from its apparent size in pixels) and steer toward
     it (FORWARD / LEFT / RIGHT).
  3. Once close enough, log the current GPS position, mark this
     waypoint done, and move on to the next one.
  4. If the target marker isn't visible, sweep left/right and forward
     to search for it.
  5. After the last waypoint, switch to GPS navigation mode and drive
     back through the logged GPS points in reverse, ending back at the
     start.

Movement/GPS requests are sent to the rover's ESP32 over serial as
short text commands (e.g. "FW\\n" = forward). The ESP32 firmware itself
(motor control, pairing with the remote) isn't part of this repo — see
the top-level README for why.

Requirements: opencv-python, numpy, picamera2, pyserial
"""

import math
import time
from pathlib import Path

import cv2
import numpy as np
import serial
from cv2 import aruco
from picamera2 import Picamera2

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200

# Where captured logs get written. Change this to wherever you want on
# your own machine — it used to be a hardcoded absolute path, which only
# ever worked on the original Pi it was written on.
SAVE_DIR = Path("./output")
SAVE_DIR.mkdir(exist_ok=True, parents=True)
LOCATION_LOG = SAVE_DIR / "waypoint_locations.txt"

# The ordered sequence of (marker ID, marker color) the rover must visit.
WAYPOINTS = [
    (11, "Red"), (40, "Green"), (17, "Blue"), (65, "Red"), (72, "Green"),
    (77, "Blue"), (93, "Red"), (93, "Green"), (1, "Blue"), (60, "Red"),
]

TURN_DURATION = 5.0   # seconds spent tank-turning while searching
MOVE_DURATION = 2.0   # seconds spent creeping forward while searching

REAL_TAG_SIZE = 0.15       # physical marker size, meters — used for distance estimation
FOCAL_LENGTH_PX = 950      # camera focal length in pixels, calibrated for this camera/lens

DISPLAY_WIDTH, DISPLAY_HEIGHT = 640, 480
CENTER_X = DISPLAY_WIDTH // 2

# Text commands understood by the rover's ESP32 firmware.
COMMANDS = {
    "TANK_TURN": b"TT\n",
    "FORWARD": b"FW\n",
    "LEFT": b"LT\n",
    "RIGHT": b"RT\n",
    "STOP": b"ST\n",
    "TANK_TURN_LEFT": b"TL\n",
    "TANK_TURN_RIGHT": b"TR\n",
    "GET_GPS": b"GS\n",
    "FORWARD_SEARCH": b"FS\n",
}

# State machine states
STATE_SEARCH, STATE_MOVE, STATE_TURN, STATE_NEXT_WP, STATE_GPS_NAVIGATION = range(5)
STATE_NAMES = ["SEARCH", "MOVE", "TURN", "NEXT_WP", "RETURNING"]


def send_command(ser, cmd):
    if ser.is_open and cmd in COMMANDS:
        ser.write(COMMANDS[cmd])


def classify_marker_color(bgr_color):
    """Classify an averaged BGR patch as Red/Green/Blue/Unknown.

    Checks saturation/value first so a dim or washed-out patch (poor
    lighting, motion blur) is reported as Unknown rather than guessed
    at — better to keep searching than to act on a bad read.
    """
    hsv = cv2.cvtColor(np.uint8([[bgr_color]]), cv2.COLOR_BGR2HSV)[0][0]
    hue, saturation, value = hsv

    if saturation < 60 or value < 50:
        return "Unknown"

    if hue <= 8 or hue >= 172:
        return "Red"
    elif 40 <= hue <= 80:
        return "Green"
    elif 95 <= hue <= 135:
        return "Blue"
    else:
        return "Unknown"


class RoverCamera:
    """Thin wrapper around the Pi camera."""

    def __init__(self, resolution=(640, 480)):
        self.picam2 = Picamera2()
        self.picam2.configure(
            self.picam2.create_preview_configuration(main={"size": resolution})
        )
        self.picam2.start()
        time.sleep(2)  # let auto-exposure settle before the first real frame

    def read(self):
        frame = self.picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return True, frame

    def release(self):
        self.picam2.stop()


def get_current_gps(ser):
    """Ask the ESP32 for its current GPS fix. Returns (lat, lon) or
    (None, None) if no fix is available yet."""
    send_command(ser, "GET_GPS")
    gps_data = ser.readline().decode(errors="ignore").strip()
    if gps_data.startswith("LAT:"):
        lat_part, lon_part = gps_data.split(",")
        return float(lat_part[4:]), float(lon_part[4:])
    return None, None


def log_current_location(ser):
    """Called once we've reached a waypoint — records where we are so
    the return trip (STATE_GPS_NAVIGATION) can retrace these points."""
    lat, lon = get_current_gps(ser)
    if lat is not None:
        with open(LOCATION_LOG, "a") as f:
            f.write(f"{lat},{lon}\n")


def load_gps_waypoints():
    if LOCATION_LOG.exists():
        with open(LOCATION_LOG) as f:
            return [tuple(map(float, line.strip().split(","))) for line in f]
    return []


def calculate_distance(lat1, lon1, lat2, lon2):
    """Great-circle distance in meters between two GPS points."""
    r = 6371000
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat, dlon = lat2 - lat1, lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * r * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def get_gps_command(current_lat, current_lon, target_lat, target_lon):
    """Very simple heading-based steering: compute the compass bearing
    to the target and turn toward it. No compass on the rover, so this
    assumes the previous FORWARD command roughly maintained heading."""
    lat1, lon1, lat2, lon2 = map(math.radians, [current_lat, current_lon, target_lat, target_lon])
    dlon = lon2 - lon1
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    bearing = (math.degrees(math.atan2(x, y)) + 360) % 360

    if bearing > 15:
        return "RIGHT"
    elif bearing < -15:
        return "LEFT"
    else:
        return "FORWARD"


def find_target_marker(frame, corners, ids, target_id, target_color):
    """Look for the marker matching (target_id, target_color) in this
    frame. Returns (distance_m, center_x_px) for the closest match, or
    None if it isn't visible / doesn't match the expected color."""
    if ids is None:
        return None

    best = None
    for i, corner in enumerate(corners):
        if int(ids[i][0]) != target_id:
            continue

        pts = corner[0].astype(int)
        pixel_width = np.linalg.norm(pts[0] - pts[1])
        if pixel_width <= 0:
            continue

        # Sample the region inside the marker's bounding box to read its color
        x1, y1 = max(pts[:, 0].min() - 10, 0), max(pts[:, 1].min() - 10, 0)
        x2 = min(pts[:, 0].max() + 10, frame.shape[1])
        y2 = min(pts[:, 1].max() + 10, frame.shape[0])
        roi = frame[y1:y2, x1:x2]
        if roi.size == 0:
            continue

        if classify_marker_color(cv2.mean(roi)[:3]) != target_color:
            continue

        distance = (FOCAL_LENGTH_PX * REAL_TAG_SIZE) / pixel_width
        cx = int(pts[:, 0].mean())
        if best is None or distance < best[0]:
            best = (distance, cx)

    return best


def main():
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
    detector = aruco.ArucoDetector(aruco_dict, aruco.DetectorParameters())

    cam = RoverCamera()
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)

    current_state = STATE_SEARCH
    current_action = "TANK_TURN"          # alternates with FORWARD_SEARCH while sweeping
    last_action_time = time.time()
    current_wp_index = 0
    last_seen_dir = None                  # which way we last saw the target, for STATE_TURN
    last_command, last_command_time = "", time.time()
    gps_waypoints, current_gps_index = [], 0

    try:
        while True:
            ret, frame = cam.read()
            if not ret:
                continue

            frame = cv2.resize(frame, (DISPLAY_WIDTH, DISPLAY_HEIGHT))
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)

            target_found = False
            command = "STOP"

            if current_wp_index < len(WAYPOINTS):
                target_id, target_color = WAYPOINTS[current_wp_index]
                match = find_target_marker(frame, corners, ids, target_id, target_color)

                if match is not None:
                    target_found = True
                    distance, cx = match
                    if distance > 0.4:
                        # Still approaching — steer toward the marker
                        if abs(cx - CENTER_X) < 40:
                            command = "FORWARD"
                        elif cx < CENTER_X - 40:
                            last_seen_dir, command = "left", "LEFT"
                        else:
                            last_seen_dir, command = "right", "RIGHT"
                    else:
                        # Close enough — log position and stop before advancing
                        log_current_location(ser)
                        command = "STOP"

            # --- state machine ---
            if current_state == STATE_SEARCH:
                if target_found:
                    current_state = STATE_MOVE
                else:
                    # Alternate tank-turning and creeping forward to sweep for the marker
                    sweep_duration = TURN_DURATION if current_action == "TANK_TURN" else MOVE_DURATION
                    if time.time() - last_action_time > sweep_duration:
                        current_action = "FORWARD_SEARCH" if current_action == "TANK_TURN" else "TANK_TURN"
                        last_action_time = time.time()
                    command = current_action

            elif current_state == STATE_MOVE:
                if not target_found:
                    current_state, last_turn_time = STATE_TURN, time.time()
                elif command == "STOP":
                    current_state = STATE_NEXT_WP

            elif current_state == STATE_TURN:
                # Lost sight of the marker while approaching — turn back toward
                # the side it was last seen on, then fall back to full search.
                if target_found:
                    current_state = STATE_MOVE
                else:
                    if last_seen_dir == "left":
                        command = "TANK_TURN_LEFT"
                    elif last_seen_dir == "right":
                        command = "TANK_TURN_RIGHT"
                    if time.time() - last_turn_time > TURN_DURATION:
                        current_state = STATE_SEARCH

            elif current_state == STATE_GPS_NAVIGATION:
                # Retrace logged waypoints in reverse order back to the start
                current_lat, current_lon = get_current_gps(ser)
                if current_lat is not None:
                    target_lat, target_lon = gps_waypoints[current_gps_index]
                    if calculate_distance(current_lat, current_lon, target_lat, target_lon) < 2.0:
                        current_gps_index -= 1
                        if current_gps_index < 0:
                            break  # reached the start — done
                    else:
                        command = get_gps_command(current_lat, current_lon, target_lat, target_lon)

            elif current_state == STATE_NEXT_WP:
                current_wp_index += 1
                if current_wp_index >= len(WAYPOINTS):
                    # All markers visited — switch to driving the GPS track home
                    gps_waypoints = load_gps_waypoints()
                    if gps_waypoints:
                        current_gps_index = len(gps_waypoints) - 1
                        current_state = STATE_GPS_NAVIGATION
                    else:
                        break
                else:
                    current_state = STATE_SEARCH

            # Only resend a command if it changed, or as a periodic keep-alive
            if command != last_command or time.time() - last_command_time > 0.5:
                send_command(ser, command)
                last_command, last_command_time = command, time.time()

            cv2.putText(frame, f"STATE: {STATE_NAMES[current_state]}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(frame, f"COMMAND: {command}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.imshow("Rover — ArUco Navigation", frame)
            if cv2.waitKey(1) == ord("q"):
                break
    finally:
        cam.release()
        ser.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
