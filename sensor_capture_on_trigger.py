"""
Triggered Photo + Sensor Capture

Day 3 was manual-drive excavator-arm skills. The driver operated the
rover by remote, and pressed a button on the remote whenever they
wanted a record of that moment: this script runs on the rover's
Raspberry Pi, waits for that button press (forwarded to it by the
ESP32 as a short serial message), and when it arrives:

  1. requests an environmental sensor reading from the ESP32
     (light level, temperature, humidity, pressure, estimated CO2),
  2. takes a photo from the Pi camera,
  3. appends a timestamped line linking the photo filename to the
     sensor reading taken at the same moment.

The remote -> ESP32 -> Pi button-press relay and the sensor hardware
itself aren't part of this repo — this script only covers the
Raspberry Pi side. See the top-level README for more on that split.

Requirements: opencv-python, picamera2, pyserial
"""

import time
from datetime import datetime
from pathlib import Path

import cv2
import serial
from picamera2 import Picamera2

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200

# Where photos and the sensor log get written. Used to be a hardcoded
# absolute path specific to the original Pi — change this to wherever
# you want on your own machine.
SAVE_DIR = Path("./output")
SAVE_DIR.mkdir(exist_ok=True, parents=True)
SENSOR_LOG = SAVE_DIR / "sensors.txt"


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

    def capture_photo(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = SAVE_DIR / f"photo_{timestamp}.jpg"
        self.picam2.capture_file(str(filename))
        return filename

    def release(self):
        self.picam2.stop()


def esp_requested_photo(ser):
    """Non-blocking check: did the ESP32 just relay a button-press from
    the remote asking for a photo?"""
    if ser.in_waiting > 0:
        line = ser.readline().decode(errors="ignore").strip()
        return "PHOTO_CMD" in line
    return False


def get_sensor_reading(ser, timeout=2.0):
    """Ask the ESP32 for a sensor reading and return its raw report
    line (e.g. "Lux:1234.5 Sıcaklık:24.1°C Basınç:1002.3hPa ...").

    The ESP32 sends the whole reading as a single line, so this just
    waits (up to `timeout` seconds) for a line starting with a sensor
    label and returns it as-is rather than trying to split it apart —
    that keeps this side simple and tolerant of the firmware's exact
    field order/format changing.
    """
    ser.write(b"GET_SENSORS\n")
    reading = ""
    start_time = time.time()
    while time.time() - start_time < timeout:
        if ser.in_waiting > 0:
            line = ser.readline().decode(errors="ignore").strip()
            if line.startswith(("Lux:", "Nem:")):
                reading += line + " | "
    return reading or "No sensor data"


def main():
    cam = RoverCamera()
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)

    try:
        while True:
            ret, frame = cam.read()
            if not ret:
                continue
            frame = cv2.resize(frame, (640, 480))

            if esp_requested_photo(ser):
                try:
                    sensor_data = get_sensor_reading(ser)
                    photo_path = cam.capture_photo()

                    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    with open(SENSOR_LOG, "a") as f:
                        f.write(f"{timestamp} | {photo_path.name} | {sensor_data}\n")

                    ser.write(b"DATA_SAVED\n")  # let the remote/ESP32 know it's done
                    print(f"Saved {photo_path.name} with sensor reading: {sensor_data}")
                except Exception as e:
                    print(f"Capture failed: {e}")
                    ser.write(b"ERROR\n")

            cv2.imshow("Rover — Camera Trigger", frame)
            if cv2.waitKey(1) == ord("q"):
                break
    finally:
        cam.release()
        ser.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
