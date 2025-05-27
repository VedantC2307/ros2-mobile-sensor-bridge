# Mobile Sensor Bridge for ROS2

This package provides a ROS2 node that streams sensor data directly from an Android smartphone, including JPEG camera streaming (up to 30 FPS), spatial pose tracking, and bidirectional audio communication.

**You can use this package in two ways:**  
- **[As a ROS2 package](#installation)**
- **[Using Docker](#docker-deployment)**

## Overview
Robotics prototypes often require multiple sensors, each needing calibration and integration. This package transforms an Android phone into a comprehensive sensor suite by publishing its sensor data as ROS2 topics. The node is implemented using rclnodejs, enabling seamless ROS2 integration within a JavaScript environment.

<img src="resources/interface.jpeg" alt="Mobile Interface" width="290">

## Features

- **Camera Stream:** Publishes mobile camera frames as `sensor_msgs/CompressedImage`
- **Native ROS2 Interface:** Direct integration with ROS2
- **Spatial Pose Tracking:** Streams WebXR spatial position and orientation as `geometry_msgs/Pose`
- **IMU Sensor Data:** Publishes accelerometer, gyroscope, and magnetometer data from iOS devices as `sensor_msgs/Imu`
- **Speech Interface:** Bidirectional audio with speech-to-text and text-to-speech (wake word: "Robot")
- **Selectable Sensors:** Enable or disable individual sensors as needed

## Prerequisites
- ROS2 (tested with Humble)
- Node.js (v20+ recommended; tested with v22.14.0)
- npm (v8+; tested with v10.9.2)
- Modern smartphone with WebXR support (for AR features)
- OpenSSL (for certificate generation)

## Installation

1. Clone the repository to your ROS2 workspace `src` directory:
   ```bash
   cd <ros2_workspace>/src/
   git clone https://github.com/VedantC2307/ros2-android-sensor-bridge.git mobile_sensor
   ```

2. Install Node.js dependencies:
   ```bash
   cd mobile_sensor
   npm install
   ```
   
> [!NOTE]
> Run `npm install` before building the package with colcon.

3. Generate SSL certificates (required for secure access):
   ```bash
   cd <ros2_workspace>/src/mobile_sensor/src
   chmod +x generate_ssl_cert.sh
   ./generate_ssl_cert.sh
   ```
   
4. Build the ROS2 package:
   ```bash
   cd <ros2_workspace>
   colcon build --packages-select mobile_sensor
   ```

5. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Configuration

You can customize the behavior of the mobile sensor node by editing the `config.yaml` file located in the src directory:

```yaml
camera:
  quality: 0.7  # Image quality (0.0 - 1.0)
  fps: 30       # Maximum frames per second (up to 30)
  facingMode: "environment"  # Camera to use: "environment" (back) or "user" (front)

microphone:
  wake_word: "robot"  # Wake word for voice commands

```

## Usage

1. Launch the mobile sensor node:
   ```bash
   ros2 launch mobile_sensor mobile_sensors.launch.py
   ```

2. Access the web interface on your mobile device:
   - Open a browser on your mobile device and navigate to `https://<your_computer_ip>:4000`
   - Accept the self-signed certificate warning
   - Grant permissions for camera, microphone, AR features, and motion/orientation (for IMU)
   - Select desired sensors and click "Start"
   
   > **Note:** IMU sensor data is currently supported only on iOS devices (iPhone/iPad). The device must grant motion and orientation permissions for IMU data collection.

## ROS2 Topics

The package publishes the following topics:

- `/camera/image_raw/compressed` (`sensor_msgs/CompressedImage`): Camera images
- `/camera/camera_info` (`sensor_msgs/CameraInfo`): Camera calibration data [testing]
- `/mobile_sensor/pose` (`geometry_msgs/Pose`): AR pose data
- `/mobile_sensor/speech` (`std_msgs/String`): Transcribed speech
- `/mobile_sensor/imu` (`sensor_msgs/Imu`): IMU data from iOS devices

To send text-to-speech messages to the device, publish to:

- `/mobile_sensor/tts` (`std_msgs/String`): Text to be spoken

### Viewing Camera Frames with rqt
To view the camera frames published by the mobile sensor node:

1. Open `rqt`:
   ```bash
   rqt
   ```

2. Navigate to **Plugins** > **Visualization** > **Image View**.

3. In the **Image View** plugin, select the topic `/camera/image_raw/compressed` to view the stream.

## Docker Deployment

For Docker deployment, see [Docker instructions](docker/README.md).

1. Build and start the Docker container:
   ```bash
   docker-compose build
   docker-compose up
   ```

2. Access the web interface on your mobile device as described in the Usage section.

## Feature Status

Current feature implementation status:
- [x] **Camera Stream** – Stable; JPEG camera streaming
- [x] **Camera Selection Flexibility** – Implemented; Supports rear or front camera selection via config.yaml
- [x] **ROS2 Topic Integration** – Stable; all core topics are fully supported
- [x] **Device Pose Tracking** – Experimental; may show reduced accuracy in visually sparse environments
- [x] **IMU Data Publishing** – Experimental; Currently supports iOS devices with accelerometer, gyroscope, and magnetometer data
- [ ] **Speech-to-Text Interface** – Experimental; works best in quiet settings. Wake word customization planned
- [ ] **Text-to-Speech Interface** – Experimental; .wav file support planned for more natural voice output
- [ ] **Multi-Device Support** – Beta; Use camera from one smartphone and audio/microphone from another
- [ ] **Dependency Management** – Planned; add rosdep or similar for improved robustness

> **Note:** Feature status is current as of April 2025. See GitHub issues for the latest updates.

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
