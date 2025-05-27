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
    imu: null // Added IMU publisher
};

// Store node reference
let rosNode = null;

// Initialize the ROS2 node and set up publishers and subscribers
async function initRos(wssTTS, wssWavAudio) {
  await rclnodejs.init();
  
  // Create the ROS node
  const node = rclnodejs.createNode('mobile_sensor_node');
  rosNode = node;
  
  // Create publishers for camera data
  publishers.compressed = node.createPublisher(
    'sensor_msgs/msg/CompressedImage',
    'camera/image_raw/compressed', // Changed to avoid topic type conflicts
    { qos: { depth: 10 } }
  );

  publishers.cameraInfo = node.createPublisher(
    'sensor_msgs/msg/CameraInfo',
    'camera/camera_info', // Updated to match the new camera namespace
    { qos: { depth: 10 } }
  );

  // Add pose publisher
  publishers.pose = node.createPublisher(
    'geometry_msgs/msg/Pose',
    'mobile_sensor/pose',
    { qos: { depth: 10 } }
  );

  // Add Microphone publisher (renamed from Audio to clarify this is input)
  publishers.microphone = node.createPublisher(
    'std_msgs/msg/String',
    'mobile_sensor/speech',
    { qos: { depth: 10 } }
  );
  
  // Add IMU publisher for iOS sensor data
  publishers.imu = node.createPublisher(
    'sensor_msgs/msg/Imu',
    'mobile_sensor/imu',
    { qos: { depth: 10 } }
  );
  
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
  
  // Publish CompressedImage message
  const compressedMsg = {
    header: header,
    format: 'jpeg',
    data: Array.from(imageBuffer)
  };
  publishers.compressed.publish(compressedMsg);
  
  // Publish CameraInfo message
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

// Method to publish IMU data from iOS sensors
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
    
    // Orientation as quaternion
    // We don't have direct quaternion data, so using placeholder values
    // A more accurate implementation would calculate quaternions from raw magnetometer data
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

module.exports = {
  initRos,
  startSpinning,
  shutdown,
  publishCameraData,
  publishPoseData,
  publishMicrophoneTranscription, // Renamed from publishAudioTranscription
  publishIMUData, // Added for iOS IMU sensor data
  getPublishers: () => publishers
};