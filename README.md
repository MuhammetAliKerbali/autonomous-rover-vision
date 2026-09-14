# Rover Project — Vision & Navigation Software

Software I wrote for a team rover research project, covering two
days of a multi-day competition. This repo contains the two pieces of
software I was personally responsible for; it does not include the
rover/remote embedded firmware, which was a team effort (see
[What's not here](#whats-not-here) below).

## What's in here

- **`aruco_autonomous_navigation.py`** The rover autonomously
  visits a sequence of ArUco markers, identified by ID and color, using
  the Pi camera for detection and distance estimation. Once every
  marker is visited, it switches to GPS mode and retraces its own
  logged path back to the starting point.

- **`sensor_capture_on_trigger.py`**  Waits for a button press on the remote (relayed
  through the rover's ESP32) and, on trigger, captures a photo
  alongside an environmental sensor reading (light, temperature,
  humidity, pressure, estimated CO2), logging both together.

Both scripts talk to the rover's ESP32 over a serial link using a
small set of text commands (e.g. `FW` = forward, `T` = take photo).

## What's not here

The ESP32 firmware — motor control and the esp_now wireless pairing
between the rover and the remote — was written and maintained by
teammates, so it isn't included in this repo. At a high level: the
rover and remote are ESP32 boards paired over esp_now; the remote
relays driver input to the
rover, which drives the motors and reports back sensor/GPS data over
serial to the Raspberry Pi running the scripts above.

## Setup

```
pip install opencv-python numpy picamera2 pyserial
python aruco_autonomous_navigation.py
python sensor_capture_on_trigger.py
```

Both scripts assume:
- a Raspberry Pi camera accessible via `picamera2`
- a serial connection to the ESP32 at `/dev/ttyUSB0`, 115200 baud
- output (photos/logs) written to `./output`, created automatically

Adjust `SERIAL_PORT` and `SAVE_DIR` at the top of each file for your
own setup.

## Background

Built as part of a university team rover research project. My focus
was the vision and autonomous navigation software — ArUco-based
marker following with distance estimation, GPS-based return-to-start,
and the sensor/camera logging used on the manual-drive day.
