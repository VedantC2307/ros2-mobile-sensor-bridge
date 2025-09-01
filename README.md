# mobile_sensor (ROS 2) – Mobile Device Sensor Bridge (v2.0)

Minimal bridge to expose a phone/tablet as a ROS 2 sensor suite (camera, IMU, GPS, speech I/O, pose) over HTTPS + WebSockets using rclnodejs.

## 1. Capabilities
Publishes:
- `/camera/image_raw/compressed` (`sensor_msgs/CompressedImage`)
- `/camera/camera_info` (`sensor_msgs/CameraInfo`)
- `/mobile_sensor/imu` (`sensor_msgs/Imu`)
- `/mobile_sensor/gps` (`sensor_msgs/NavSatFix`)
- `/mobile_sensor/pose` (`geometry_msgs/Pose`)
- `/mobile_sensor/speech` (speech transcription, `std_msgs/String`)

Subscribes:
- `/mobile_sensor/tts` (`std_msgs/String`) – plain text TTS
- `/mobile_sensor/tts_wav` (`std_msgs/UInt8MultiArray`) – raw WAV bytes
- `/mobile_sensor/wav_bytes` (`std_msgs/UInt8MultiArray`) – legacy WAV topic

Features:
- iOS and Android (modern browser; iOS 13+, Android 8+)
- JPEG camera streaming (configurable FPS/quality, front/back selection)
- IMU (accelerometer + gyro)
- GPS (NavSatFix with diagonal covariance from accuracy)
- Optional WebXR pose (published as `geometry_msgs/Pose` when available)
- Wake‑word gated microphone transcription → `/mobile_sensor/speech`
- Text‑to‑speech + WAV audio playback to device
- YAML central configuration
- Self‑signed HTTPS for permission unlock (camera/mic/motion)

## 2. What’s New in 2.0
- iOS support (camera, mic, motion sensors, GPS)
- Added GPS (`/mobile_sensor/gps`)
- Added IMU publisher (`/mobile_sensor/imu`)
- Unified speech topic (`/mobile_sensor/speech`)
- WAV + text TTS channels
- Config-driven parameters (`config/config.yaml`)
- Refactored server layout (`src/server/*`)

## 3. Requirements
- ROS 2 Humble+
- Node.js 20+
- OpenSSL (for certificate generation)
- Shared Wi‑Fi network between device and ROS host

## 4. Installation
```bash
cd <ros2_ws>/src
git clone https://github.com/VedantC2307/ros2-mobile-sensor-bridge.git mobile_sensor
cd mobile_sensor
npm install            # install node dependencies BEFORE colcon build
cd src
chmod +x generate_ssl_cert.sh
./generate_ssl_cert.sh # creates ssl/key.pem + cert.pem
cd ~/<ros2_ws>         # back to workspace root
colcon build --packages-select mobile_sensor
source install/setup.bash
```

## 5. Launch
```bash
ros2 launch mobile_sensor mobile_sensors.launch.py
```
Console prints the HTTPS URL (e.g. `https://<host_ip>:4000`).

## 6. Connect Device
1. Open the printed URL in mobile browser (allow self‑signed cert).
2. Grant permissions: camera, microphone, location, motion sensors.
3. Select sensors and start streaming.


## 7. Typical Commands
Publish TTS text:
```bash
ros2 topic pub -1 /mobile_sensor/tts std_msgs/msg/String "{data: 'Hello from ROS'}"
```
Play WAV (UInt8MultiArray of bytes): ensure proper WAV header.

Monitor:
```bash
ros2 topic hz /camera/image_raw/compressed
ros2 topic echo /mobile_sensor/imu
ros2 topic echo /mobile_sensor/gps
```
View camera:
```bash
ros2 run rqt_image_view rqt_image_view
```

## 8. Topic Summary
| Topic | Type | Direction | Notes |
|-------|------|-----------|-------|
| /camera/image_raw/compressed | sensor_msgs/CompressedImage | publish | JPEG frames |
| /camera/camera_info | sensor_msgs/CameraInfo | publish | Basic intrinsics (identity defaults) |
| /mobile_sensor/imu | sensor_msgs/Imu | publish | Accel + gyro; orientation = identity |
| /mobile_sensor/gps | sensor_msgs/NavSatFix | publish | Covariance diagonal from accuracy |
| /mobile_sensor/pose | geometry_msgs/Pose | publish | Provided if WebXR pose available |
| /mobile_sensor/speech | std_msgs/String | publish | Wake‑word gated transcription |
| /mobile_sensor/tts | std_msgs/String | subscribe | Text to device TTS |
| /mobile_sensor/tts_wav | std_msgs/UInt8MultiArray | subscribe | WAV audio bytes |
| /mobile_sensor/wav_bytes | std_msgs/UInt8MultiArray | subscribe | Legacy WAV channel |

## 9. Docker (Optional)
```bash
docker-compose build
docker-compose up
# or
docker build -t mobile-sensor-bridge -f docker/Dockerfile .
docker run -p 4000:4000 -p 3000:3000 mobile-sensor-bridge
```

## 10. Troubleshooting

### Mobile Browser Shows ERR_EMPTY_RESPONSE

If the UI loads fine on your laptop but your mobile browser shows ERR_EMPTY_RESPONSE, this is usually a network visibility or firewall issue. Try the following steps:

1. **Confirm Both Devices Are on the Same Network:**
   - Ensure your phone and computer are connected to the same Wi-Fi network.
2. **Disable the Firewall Temporarily:**
   - On Ubuntu, run: `sudo ufw disable`
   - Try accessing the server again from your phone. If it works, the firewall was blocking the connection. Re-enable it using `sudo ufw enable`and add a rule to allow traffic on port 4000.

Let us know if these steps help or if you’re still having issues after trying them!

## Contributing
Contributions are welcome! Please feel free to submit a Pull Request.
