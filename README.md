# Mobile Sensor Bridge for ROS2 v2.0 🚀

[![Version](https://img.shields.io/badge/version-2.0.0-blue.svg)](https://github.com/VedantC2307/ros2-mobile-sensor-bridge)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-green.svg)](https://docs.ros.org/en/humble/)
[![Node.js](https://img.shields.io/badge/Node.js-20+-green.svg)](https://nodejs.org/)


Rapid ROS 2 prototyping bridge: drop any iOS/Android phone or tablet into your ros2 stack and instantly stream camera (JPEG), IMU, GPS, magnetometer, WebXR pose, speech transcription and text-to-speech over a secure HTTPS/WebSocket pipeline powered by rclnodejs — no native app install, just a browser, a YAML config, and one launch. Built for quick perception/audio experiments, field demos, and early integration work with configurable parameters, standardized message types, and a minimal surface for extension.

## What's New in 2.0
- iOS support
- Added GPS (`/mobile_sensor/gps`)
- Added IMU publisher (`/mobile_sensor/imu`)
- Unified speech topic (`/mobile_sensor/speech`)
- WAV + text TTS channels
- Config-driven parameters (`config/config.yaml`)
- Reorganized project layout for easier customization and extension
- Added Magnetometer (`/mobile_sensor/magnetometer`)

## Features
- Supports iOS and Android (iOS 13+, Android 8+)
- JPEG camera streaming (configurable FPS/quality, front/back camera selection)
- IMU (accelerometer + gyroscope)
- GPS (longitude, latitude and altitude)
- Magnetometer (raw magnetic field vector on Android; compass heading on iOS)
- Optional WebXR pose
- Configurable wake-word microphone transcription → `/mobile_sensor/speech`
- Native text-to-speech or `.wav` audio playback to device
- YAML central configuration

## Published Topics

| Topic | Message Type | Notes |
|-------|-------------|-------|
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | JPEG frames at configured FPS |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Basic intrinsics (identity defaults) |
| `/mobile_sensor/imu` | `sensor_msgs/Imu` | Accelerometer + gyroscope; orientation = identity quaternion |
| `/mobile_sensor/gps` | `sensor_msgs/NavSatFix` | Lat/long/alt; covariance diagonal from accuracy field |
| `/mobile_sensor/magnetometer` | `sensor_msgs/MagneticField` | See Magnetometer section below |
| `/mobile_sensor/pose` | `geometry_msgs/Pose` | WebXR viewer pose (when 3D Position enabled) |
| `/mobile_sensor/speech` | `std_msgs/String` | Wake-word gated speech transcription |

## Subscribed Topics

| Topic | Message Type | Notes |
|-------|-------------|-------|
| `/mobile_sensor/tts` | `std_msgs/String` | Plain text forwarded to device TTS |
| `/mobile_sensor/tts_wav` | `std_msgs/UInt8MultiArray` | WAV audio bytes played on device |
| `/mobile_sensor/wav_bytes` | `std_msgs/UInt8MultiArray` | Legacy WAV channel |

---

## Magnetometer

The bridge streams magnetic field data from the phone's built-in magnetometer and publishes it as a standard `sensor_msgs/msg/MagneticField` message on `/mobile_sensor/magnetometer`.

### Data source by platform

| Platform | API used | Data |
|----------|----------|------|
| **Android** (Chrome) | [Generic Sensor API](https://developer.mozilla.org/en-US/docs/Web/API/Magnetometer) | Raw x, y, z components in **Tesla** |
| **iOS** (Safari) | `DeviceOrientationEvent` / `webkitCompassHeading` | Unit direction vector derived from compass heading (magnitude unknown) |

Because iOS does not expose raw magnetometer components, the iOS fallback produces a unit vector in the XY plane pointing toward magnetic North:

```
x = sin(heading_rad)
y = cos(heading_rad)
z = 0
```

The `magnetic_field_covariance` field encodes data quality:
- `covariance[0] = -1`, rest `0` — Android Generic Sensor API (real field, unknown variance)
- All `-1` — iOS orientation fallback (direction only, magnitude unknown)

### Configuration

In `config/config.yaml`:
```yaml
magnetometer:
  sample_rate: 10   # Hz (recommended: 5–20 Hz)
```

### Monitor the topic
```bash
ros2 topic echo /mobile_sensor/magnetometer
ros2 topic hz /mobile_sensor/magnetometer
```

### Analyze with the included script
```bash
# default 30-sample sliding window
python3 scripts/magnetometer_analyzer.py

# wider window, no color output
python3 scripts/magnetometer_analyzer.py --window 100 --no-color
```

The analyzer prints live per-axis statistics (min/max/avg/std), total field magnitude, compass heading with cardinal label, and message rate.

---

## Prerequisites

### Requirements
- ROS 2 Humble [Tested]
- Node.js 20+
- OpenSSL (for certificate generation)
- Shared Wi-Fi network between device and ROS host

### Installation
```bash
cd <ros2_ws>/src
git clone https://github.com/VedantC2307/ros2-mobile-sensor-bridge.git mobile_sensor
cd mobile_sensor
export ROS_DISTRO=humble # ensure ROS environment is detected by rclnodejs
npm install              # install node dependencies BEFORE colcon build
cd src
chmod +x generate_ssl_cert.sh
mkdir -p ssl            # create ssl directory if it doesn't exist
./generate_ssl_cert.sh  # create ssl/key.pem + cert.pem
cd ~/<ros2_ws>          # back to workspace root
colcon build --packages-select mobile_sensor
source install/setup.bash
```

### Launch
```bash
ros2 launch mobile_sensor mobile_sensors.launch.py
```
Console prints the HTTPS URL (e.g. `https://<host_ip>:4000`).

### Connect Device
1. Open the printed URL in mobile browser (allow self-signed cert).
2. Grant permissions: camera, microphone, location, motion sensors.
3. Select sensors and press **Start**.

---

## Usual Commands

Publish TTS text:
```bash
ros2 topic pub -1 /mobile_sensor/tts std_msgs/msg/String "{data: 'Hello from ROS'}"
```

Monitor sensors:
```bash
ros2 topic hz /camera/image_raw/compressed
ros2 topic echo /mobile_sensor/imu
ros2 topic echo /mobile_sensor/gps
ros2 topic echo /mobile_sensor/magnetometer
```

View camera stream:
```bash
ros2 run rqt_image_view rqt_image_view
```

---

## Troubleshooting

### Mobile Browser Shows ERR_EMPTY_RESPONSE

If the UI loads fine on your laptop but your mobile browser shows ERR_EMPTY_RESPONSE, this is usually a network visibility or firewall issue. Try the following steps:

1. **Confirm Both Devices Are on the Same Network:**
   - Ensure your phone and computer are connected to the same Wi-Fi network.
2. **Disable the Firewall Temporarily:**
   - On Ubuntu, run: `sudo ufw disable`
   - Try accessing the server again from your phone. If it works, the firewall was blocking the connection. Re-enable it using `sudo ufw enable` and add a rule to allow traffic on port 4000.

Let us know if these steps help or if you're still having issues after trying them!

## Contributing
Contributions are welcome! Please feel free to submit a Pull Request.

---

<div align="center">

**Made with ❤️ for the Robotics Community**

[🌟 Star this repo](https://github.com/VedantC2307/ros2-mobile-sensor-bridge) • [🐛 Report Bug](https://github.com/VedantC2307/ros2-mobile-sensor-bridge/issues) • [💡 Request Feature](https://github.com/VedantC2307/ros2-mobile-sensor-bridge/issues/new)

</div>
