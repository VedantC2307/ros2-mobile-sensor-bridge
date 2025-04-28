/**
 * ROS Interface for the mobile sensor bridge
 * Handles all ROS2 publishers and subscribers
 */
const rclnodejs = require('rclnodejs');
const WebSocket = require('ws');

// Store all publishers for access from other modules
let publishers = {
    compressed: null,
    cameraInfo: null,
    pose: null,
    microphone: null // Changed from audio to microphone
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
    'camera/image_raw/compressed',
    { qos: { depth: 10 } }
  );

  publishers.cameraInfo = node.createPublisher(
    'sensor_msgs/msg/CameraInfo',
    'camera/camera_info',
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
  
  // Add string subscriber for TTS
  node.createSubscription(
    'std_msgs/msg/String',
    'mobile_sensor/tts',
    (msg) => {
      console.log('Received TTS message:', msg.data);
      
      // Count active clients
      let activeClientCount = 0;
      wssTTS.clients.forEach(client => {
        if (client.readyState === WebSocket.OPEN) {
          activeClientCount++;
        }
      });
      
      console.log(`TTS clients connected: ${activeClientCount}`);
      
      // Forward the message to all connected TTS clients
      wssTTS.clients.forEach(client => {
        if (client.readyState === WebSocket.OPEN) {
          client.send(msg.data);
        }
      });
    },
    { qos: { depth: 10 } }
  );
  
  // Update UInt8MultiArray subscriber for WAV audio to handle both 'wav_bytes' and 'tts_wav' topics
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
          console.log(`Forwarded WAV audio: ${buffer.length} bytes to ${clientCount} clients, WAV header: ${hasWavHeader ? 'present' : 'absent'}`);
        }
      } catch (error) {
        console.error('Error processing audio data:', error);
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
        console.log('Received wav_bytes message:', buffer.length);
        
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
          console.log(`Forwarded audio data: ${buffer.length} bytes to ${clientCount} clients, WAV header: ${hasWavHeader ? 'present' : 'absent'}`);
        }
      } catch (error) {
        console.error('Error processing audio data:', error);
      }
    },
    { qos: { depth: 10 } }
  );
  
  console.log('ROS2 node initialized with camera, pose publisher, TTS and WAV audio subscribers');
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
  if (rclnodejs.isShutdown() === false) {
    rclnodejs.shutdown();
    console.log('ROS2 node shut down');
    return true;
  }
  return false;
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

module.exports = {
  initRos,
  startSpinning,
  shutdown,
  publishCameraData,
  publishPoseData,
  publishMicrophoneTranscription, // Renamed from publishAudioTranscription
  getPublishers: () => publishers
};