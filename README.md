# Mobile Sensor Bridge for ROS2 v2.0 🚀

[![Version](https://img.shields.io/badge/version-2.0.0-blue.svg)](https://github.com/VedantC2307/ros2-mobile-sensor-bridge)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-green.svg)](https://docs.ros.org/en/humble/)
[![Node.js](https://img.shields.io/badge/Node.js-20+-green.svg)](https://nodejs.org/)

Transform your mobile device into a sensor suite for robotics development. This ROS2 package streams camera feeds, IMU data, GPS coordinates, spatial pose tracking, and audio communication from your smartphone to ROS2 topics.

<img src="resources/interface.jpeg" alt="Mobile Interface v2.0" width="300">

## What's New in v2.0

### Major feature added native IOS support. The package now supports both Android/IOS devices.
By IOS I mean iPAD and iPhone.

### Other Features Added
- **Configuration System**: YAML config file to edit all sensor parameters
- **GPS Integration**: Real-time location data as `sensor_msgs/NavSatFix`
- **IMU Sensor Support**: Accelerometer and gyroscope data as `sensor_msgs/Imu`
- **Front/Rear Camera Selection**: Choose front or rear camera via config
- **Audio File Support**: Send .wav and .mp3 files sent via ROS2 messages to play on your device
- **Enhanced Text-to-Speech**: Pitch, rate and volume control
- **Debug Console**: Mobile debug console and logging options
- **Modular Architecture**: Easy to extend for custom sensors


## Key Features

| Feature | Status | Description |
|---------|--------|-------------|
| **Camera Streaming** | ✅ Enhanced | JPEG streaming up to 30 FPS with front/rear selection |
| **GPS Location** | ✅ New in v2.0 | Real-time GPS coordinates |
| **IMU Sensors** | ✅ New in v2.0 | Accelerometer, gyroscope |
| **Spatial Pose Tracking** | ✅ Enhanced | WebXR-based 6DOF pose estimation |
| **Speech Recognition** | ✅ Customizable | Customizable wake word |
| **Text-to-Speech** | ✅ Enhanced | Voice selection and speech parameters |
| **Audio File Support** | ✅ New in v2.0 | Play .wav and .mp3 files via ROS2 messages |
| **Configuration System** | ✅ New in v2.0 | YAML-based configuration for all sensors |
| **Modular Architecture** | ✅ New in v2.0 | Easy to extend for custom sensors |

## Overview
Robotics prototypes often require multiple sensors, each needing calibration and integration. This package transforms your mobile device into a comprehensive sensor suite by publishing its sensor data as ROS2 topics. The node is implemented using rclnodejs, enabling seamless ROS2 integration within a JavaScript environment.

## Prerequisites

### Software Prerequisites
- **ROS2**: Humble or later
- **Node.js**: v20.0+ (tested with v22.14.0)
- **npm**: v8.0+ (tested with v10.9.2)
- **OpenSSL**: For SSL certificate generation

### Device Compatibility
- **Mobile Devices**: iOS 13+, Android 8.0+ with modern browser
- **Desktop**: For development and testing (limited sensor support)
- **Network**: WiFi connection between mobile device and ROS2 host

## Installation

### 1. Clone the Repository
```bash
cd <your_ros2_workspace>/src/
git clone https://github.com/VedantC2307/ros2-mobile-sensor-bridge.git mobile_sensor
```

### 2. Install Dependencies
```bash
cd mobile_sensor
npm install
```

> **Note**: Run `npm install` before building with colcon.

### 3. Generate SSL Certificates
```bash
cd src/
chmod +x generate_ssl_cert.sh
./generate_ssl_cert.sh
```

### 4. Build the ROS2 Package
```bash
cd <your_ros2_workspace>
colcon build --packages-select mobile_sensor
source install/setup.bash
```

## Configuration

Edit `config/config.yaml` to customize your setup, if needed.


## 🚀 Quick Start

### 1. Launch the Sensor Bridge
```bash
ros2 launch mobile_sensor mobile_sensors.launch.py
```

### 2. Connect Your Mobile Device
1. **Open Browser**: Navigate to `https://<your_computer_ip>:4000` on your mobile device
2. **Accept Certificate**: Accept the self-signed SSL certificate warning
3. **Grant Permissions**: Allow access to:
   - 📷 Camera (for video streaming)
   - 🎤 Microphone (for speech recognition)
   - 🧭 Location (for GPS data)
   - 📐 Motion Sensors (for IMU data - iOS only)
   - 🥽 WebXR (for pose tracking)
4. **Select Sensors**: Choose which sensors to activate
5. **Start Streaming**: Click "Start" to begin data transmission

### 3. Verify Data Streams
```bash
# Check available topics
ros2 topic list

# Monitor camera stream
ros2 topic hz /camera/image_raw/compressed

# View IMU data
ros2 topic echo /mobile_sensor/imu

```

## ROS2 Topic Reference
### Published Topics

| Topic | Message Type | Description | Frequency |
|-------|-------------|-------------|-----------|
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | Camera frames in JPEG format | Up to 30 Hz |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Camera calibration parameters | On demand |
| `/mobile_sensor/pose` | `geometry_msgs/Pose` | WebXR spatial pose (6DOF) | 60 Hz |
| `/mobile_sensor/imu` | `sensor_msgs/Imu` | IMU sensor data | 30 Hz |
| `/mobile_sensor/gps` | `sensor_msgs/NavSatFix` | GPS location coordinates | 1 Hz |
| `/mobile_sensor/speech` | `std_msgs/String` | Transcribed speech text | On speech |

### Subscribed Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/mobile_sensor/tts` | `std_msgs/String` | Text-to-speech output |

### Example Usage
```bash
# Send text-to-speech command
ros2 topic pub -1 /mobile_sensor/tts std_msgs/msg/String "{data: 'Hey! Hows it going? I am rolle'}"

# Monitor all sensor data
ros2 topic echo /mobile_sensor/imu
ros2 topic echo /mobile_sensor/gps

# To view camera
ros2 run rqt_image_view rqt_image_view
```

## 🐳 Docker Deployment

For containerized deployment with all dependencies included:

```bash
# Build and run with Docker Compose
docker-compose build
docker-compose up

# Or build manually
docker build -t mobile-sensor-bridge -f docker/Dockerfile .
docker run -p 4000:4000 -p 3000:3000 mobile-sensor-bridge
```

Access the interface at `https://<docker_host_ip>:4000` from your mobile device.


## 🔍 Troubleshooting


## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## 📜 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- **ROS2 Community**: For the excellent robotics framework
- **rclnodejs Team**: For JavaScript ROS2 integration
- **WebRTC Community**: For web-based real-time communication standards
- **Contributors**: Special thanks to all contributors who made v2.0 possible

## 📞 Support & Community

- **🐛 Issues**: [GitHub Issues](https://github.com/VedantC2307/ros2-mobile-sensor-bridge/issues)
- **💡 Feature Requests**: [GitHub Discussions](https://github.com/VedantC2307/ros2-mobile-sensor-bridge/discussions)
- **📧 Contact**: vedantchoudhary16@gmail.com
- **⭐ Star**: If this project helped you, please give it a star!

---

<div align="center">

**Made with ❤️ for the Robotics Community**

[🌟 Star this repo](https://github.com/VedantC2307/ros2-mobile-sensor-bridge) • [🐛 Report Bug](https://github.com/VedantC2307/ros2-mobile-sensor-bridge/issues) • [💡 Request Feature](https://github.com/VedantC2307/ros2-mobile-sensor-bridge/issues/new)

</div>

## Troubleshooting

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
