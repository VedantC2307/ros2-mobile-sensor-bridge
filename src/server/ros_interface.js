/**
 * ROS Interface for the mobile sensor bridge
 * Handles all ROS2 publishers and subscribers
 */
const rclnodejs = require('rclnodejs');
const WebSocket = require('ws');
const Logger = require('./logger');

// Store all publishers for access from other modules
let publishers = {
    compressed: null,
    cameraInfo: null,
    pose: null,
    microphone: null, // Changed from audio to microphone
    imu: null, // Added IMU publisher
    gps: null  // Added GPS publisher
};

// Store node reference
let rosNode = null;

// Store configuration for conditional publishing
let publishConfig = null;

// Initialize the ROS2 node and set up publishers and subscribers
async function initRos(wssTTS, wssWavAudio, config = null) {
  await rclnodejs.init();
  
  // Store the configuration for conditional publishing
  publishConfig = config;
  
  // Create the ROS node
  const node = rclnodejs.createNode('mobile_sensor_node');
  rosNode = node;
  
  // Get publish configuration, default to all enabled if not specified
  const publishSensors = (publishConfig && publishConfig.mobile_sensors && publishConfig.mobile_sensors.published_sensors) || {
    camera: true,
    pose: true,
    microphone: true,
    imu: true,
    gps: true,
    audio: true
  };
  
  // Create publishers conditionally based on configuration
  if (publishSensors.camera) {
    publishers.compressed = node.createPublisher(
      'sensor_msgs/msg/CompressedImage',
      'camera/image_raw/compressed',
      { qos: { depth: 10 } }
    );

    publishers.cameraInfo = node.createPublisher(
      'sensor_msgs/msg/CameraInfo',
      'camera/camera_info',
      { qos: { depth: 10 } }
    );
    Logger.info('ROS', 'Camera publishers created');
  } else {
    Logger.info('ROS', 'Camera publishing disabled in configuration');
  }

  if (publishSensors.pose) {
    publishers.pose = node.createPublisher(
      'geometry_msgs/msg/Pose',
      'mobile_sensor/pose',
      { qos: { depth: 10 } }
    );
    Logger.info('ROS', 'Pose publisher created');
  } else {
    Logger.info('ROS', 'Pose publishing disabled in configuration');
  }

  if (publishSensors.microphone) {
    publishers.microphone = node.createPublisher(
      'std_msgs/msg/String',
      'mobile_sensor/speech',
      { qos: { depth: 10 } }
    );
    Logger.info('ROS', 'Microphone publisher created');
  } else {
    Logger.info('ROS', 'Microphone publishing disabled in configuration');
  }
  
  if (publishSensors.imu) {
    publishers.imu = node.createPublisher(
      'sensor_msgs/msg/Imu',
      'mobile_sensor/imu',
      { qos: { depth: 10 } }
    );
    Logger.info('ROS', 'IMU publisher created');
  } else {
    Logger.info('ROS', 'IMU publishing disabled in configuration');
  }
  
  if (publishSensors.gps) {
    publishers.gps = node.createPublisher(
      'sensor_msgs/msg/NavSatFix',
      'mobile_sensor/gps',
      { qos: { depth: 10 } }
    );
    Logger.info('ROS', 'GPS publisher created');
  } else {
    Logger.info('ROS', 'GPS publishing disabled in configuration');
  }
  
  // Add string subscriber for TTS
  node.createSubscription(
    'std_msgs/msg/String',
    'mobile_sensor/tts',
    (msg) => {
      Logger.info('ROS', `Received TTS message: ${msg.data}`);
      
      // Count active clients
      let activeClientCount = 0;
      wssTTS.clients.forEach(client => {
        if (client.readyState === WebSocket.OPEN) {
          activeClientCount++;
        }
      });
      
      Logger.debug('ROS', `TTS clients connected: ${activeClientCount}`);
      
      // Forward the message to all connected TTS clients
      wssTTS.clients.forEach(client => {
        if (client.readyState === WebSocket.OPEN) {
          client.send(msg.data);
        }
      });
    },
    { qos: { depth: 10 } }
  );
  
  // Add UInt8MultiArray subscriber for WAV audio to handle both 'wav_bytes' and 'tts_wav' topics
  node.createSubscription(
    'std_msgs/msg/UInt8MultiArray',
    'mobile_sensor/tts_wav',
    (msg) => {
      try {
        const buffer = Buffer.from(msg.data);
        
        // Check if the buffer has a WAV header (starts with 'RIFF' and contains 'WAVE')
        const hasWavHeader = 
          buffer.length >= 12 && 
          buffer[0] === 82 && buffer[1] === 73 && buffer[2] === 70 && buffer[3] === 70 && // 'RIFF'
          buffer[8] === 87 && buffer[9] === 65 && buffer[10] === 86 && buffer[11] === 69; // 'WAVE'
        
        // Send the buffer to all connected wav_audio clients
        let clientCount = 0;
        
        wssWavAudio.clients.forEach(client => {
          if (client.readyState === WebSocket.OPEN) {
            client.send(buffer);
            clientCount++;
          }
        });
        
        if (clientCount > 0) {
          Logger.debug('ROS', `Forwarded WAV audio: ${buffer.length} bytes to ${clientCount} clients.`);
        }
      } catch (error) {
        Logger.error('ROS', `Error processing audio data: ${error}`);
      }
    },
    { qos: { depth: 10 } }
  );
  
  // Also keep the original wav_bytes subscription for backward compatibility
  node.createSubscription(
    'std_msgs/msg/UInt8MultiArray',
    'mobile_sensor/wav_bytes',
    (msg) => {
      try {
        const buffer = Buffer.from(msg.data);
        
        // Check if the buffer has a WAV header (starts with 'RIFF' and contains 'WAVE')
        const hasWavHeader = 
          buffer.length >= 12 && 
          buffer[0] === 82 && buffer[1] === 73 && buffer[2] === 70 && buffer[3] === 70 && // 'RIFF'
          buffer[8] === 87 && buffer[9] === 65 && buffer[10] === 86 && buffer[11] === 69; // 'WAVE'
        
        // Send the buffer to all connected wav_audio clients
        let clientCount = 0;
        
        wssWavAudio.clients.forEach(client => {
          if (client.readyState === WebSocket.OPEN) {
            client.send(buffer);
            clientCount++;
          }
        });
        
        if (clientCount > 0) {
          Logger.debug('ROS', `Forwarded WAV audio: ${buffer.length} bytes to ${clientCount} clients, WAV header: ${hasWavHeader ? 'present' : 'absent'}`);
        }
      } catch (error) {
        Logger.error('ROS', `Error processing audio data: ${error}`);
      }
    },
    { qos: { depth: 10 } }
  );
  
  // console.log('ROS2 node initialized with camera, pose publisher, STT publishers and audio subscribers');
  return node;
}

// Method to start spinning the ROS node
function startSpinning() {
  if (rosNode) {
    rclnodejs.spin(rosNode);
    return true;
  }
  return false;
}

// Method to shut down ROS node
function shutdown() {
  return new Promise((resolve) => {
    Logger.info('ROS', 'Shutting down ROS2 node...');
    
    if (rclnodejs.isShutdown() === false) {
      try {
        // Clear any remaining callbacks
        if (rosNode) {
          for (const pub in publishers) {
            if (publishers[pub]) {
              publishers[pub] = null;
            }
          }
        }
        
        // Shutdown the node
        rclnodejs.shutdown();
        Logger.success('ROS', 'ROS2 node shut down successfully');
      } catch (error) {
        Logger.error('ROS', `Error during ROS2 shutdown: ${error}`);
      }
    } else {
      Logger.info('ROS', 'ROS2 already shut down');
    }
    
    // Always resolve to prevent hanging
    resolve(true);
  });
}

// Method to publish camera data
function publishCameraData(imageBuffer, width, height, timestamp) {
  if (!publishers.compressed || !publishers.cameraInfo) return false;
  
  // Generate standard header
  const header = {
    stamp: timestamp || {
      sec: Math.floor(Date.now() / 1000),
      nanosec: (Date.now() % 1000) * 1000000
    },
    frame_id: 'camera_frame'
  };
  
  // Publish CompressedImage message only if publisher exists
  if (publishers.compressed) {
    const compressedMsg = {
      header: header,
      format: 'jpeg',
      data: Array.from(imageBuffer)
    };
    publishers.compressed.publish(compressedMsg);
  }
  
  // Publish CameraInfo message only if publisher exists
  if (publishers.cameraInfo) {
    const cameraInfoMsg = {
      header: header,
      height: height,
      width: width,
      distortion_model: 'plumb_bob',
      d: [0.0, 0.0, 0.0, 0.0, 0.0],  // Default distortion coefficients
      k: [  // Default intrinsic camera matrix
        width, 0.0, width/2,
        0.0, height, height/2,
        0.0, 0.0, 1.0
      ],
      r: [  // Default rectification matrix (identity)
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0
    ],
    p: [  // Default projection matrix
      width, 0.0, width/2, 0.0,
      0.0, height, height/2, 0.0,
      0.0, 0.0, 1.0, 0.0
    ],
    binning_x: 0,
    binning_y: 0,
    roi: {
      x_offset: 0,
      y_offset: 0,
      height: height,
      width: width,
      do_rectify: false
    }
    };
    publishers.cameraInfo.publish(cameraInfoMsg);
  }
  
  return true;
}

// Method to publish pose data
function publishPoseData(poseData, timestamp) {
  if (!publishers.pose) return false;
  
  const poseMsg = {
    position: {
      x: poseData.position.x,
      y: poseData.position.y,
      z: poseData.position.z
    },
    orientation: {
      x: poseData.orientation.x,
      y: poseData.orientation.y,
      z: poseData.orientation.z,
      w: poseData.orientation.w
    }
  };
  publishers.pose.publish(poseMsg);
  
  return true;
}

// Method to publish speech transcription (renamed from publishAudioTranscription)
function publishMicrophoneTranscription(transcription, timestamp) {
  if (!publishers.microphone) return false;
  
  // Create timestamped message with header
  const msg = {
    header: {
      stamp: timestamp || {
        sec: Math.floor(Date.now() / 1000),
        nanosec: (Date.now() % 1000) * 1000000
      },
      frame_id: 'microphone_frame'
    },
    data: transcription
  };
  publishers.microphone.publish(msg);
  
  return true;
}

// Method to publish IMU data from iOS and Android sensors
function publishIMUData(imuData, timestamp) {
  if (!publishers.imu) return false;
  
  // Generate standard header
  const header = {
    stamp: timestamp || {
      sec: Math.floor(Date.now() / 1000),
      nanosec: (Date.now() % 1000) * 1000000
    },
    frame_id: 'imu_frame'
  };
  
  // Create IMU message according to sensor_msgs/msg/Imu format
  // http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html
  const imuMsg = {
    header: header,
    
    // Linear acceleration in m/s^2
    linear_acceleration: {
      x: imuData.accelerometer.x || 0.0,
      y: imuData.accelerometer.y || 0.0,
      z: imuData.accelerometer.z || 0.0
    },
    
    // Angular velocity in rad/s
    // alpha (z), beta (x), gamma (y) from gyroscope data
    angular_velocity: {
      x: imuData.gyroscope.beta || 0.0,  // beta is rotation around x-axis
      y: imuData.gyroscope.gamma || 0.0, // gamma is rotation around y-axis
      z: imuData.gyroscope.alpha || 0.0  // alpha is rotation around z-axis
    },
    
    // Set default identity quaternion for orientation
    // We're no longer collecting orientation data
    orientation: {
      x: 0.0,
      y: 0.0,
      z: 0.0,
      w: 1.0
    },
    
    // Covariance matrices (9x9 arrays) - using default values
    // -1 indicates that the covariance is unknown
    orientation_covariance: new Array(9).fill(-1),
    angular_velocity_covariance: new Array(9).fill(-1),
    linear_acceleration_covariance: new Array(9).fill(-1)
  };
  
  publishers.imu.publish(imuMsg);
  return true;
}

// Method to publish GPS data using sensor_msgs/NavSatFix
function publishGPSData(gpsData, timestamp) {
  if (!publishers.gps) return false;
  
  // Generate standard header
  const header = {
    stamp: timestamp || {
      sec: Math.floor(Date.now() / 1000),
      nanosec: (Date.now() % 1000) * 1000000
    },
    frame_id: 'gps_frame'
  };
  
  // Handle altitude - can be null from geolocation API
  const altitude = (gpsData.altitude !== null && !isNaN(gpsData.altitude)) ? gpsData.altitude : 0.0;
  
  // Convert accuracy to covariance
  // GPS accuracy is typically in meters, we'll use it for position covariance
  const accuracy = gpsData.accuracy || 1.0; // Default to 1 meter if not provided
  const variance = accuracy * accuracy; // Convert to variance (accuracy squared)
  
  // Create position covariance matrix (3x3 matrix flattened to 9 elements)
  // Only fill diagonal elements [0,4,8] with variance, rest are 0
  const positionCovariance = new Array(9).fill(0.0);
  positionCovariance[0] = variance; // x variance (longitude)
  positionCovariance[4] = variance; // y variance (latitude)
  positionCovariance[8] = variance; // z variance (altitude)
  
  // Create NavSatFix message according to sensor_msgs/msg/NavSatFix format
  // http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/NavSatFix.html
  const navSatFixMsg = {
    header: header,
    
    // Navigation satellite fix status
    status: {
      status: 0,    // STATUS_FIX (0 = fix, -1 = no fix)
      service: 1    // SERVICE_GPS (1 = GPS, 2 = GLONASS, 4 = COMPASS, 8 = GALILEO)
    },
    
    // Position in degrees
    latitude: gpsData.latitude || 0.0,
    longitude: gpsData.longitude || 0.0,
    altitude: altitude,
    
    // Position covariance matrix
    position_covariance: positionCovariance,
    position_covariance_type: 2  // COVARIANCE_TYPE_DIAGONAL_KNOWN
  };
  
  publishers.gps.publish(navSatFixMsg);
  return true;
}

module.exports = {
  initRos,
  startSpinning,
  shutdown,
  publishCameraData,
  publishPoseData,
  publishMicrophoneTranscription, // Renamed from publishAudioTranscription
  publishIMUData, // Added for iOS IMU sensor data
  publishGPSData, // Added for GPS location data
  getPublishers: () => publishers
};