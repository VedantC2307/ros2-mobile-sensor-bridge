const https = require('https');
const fs = require('fs');
const express = require('express');
const path = require('path');
const WebSocket = require('ws');
const rclnodejs = require('rclnodejs');
const yaml = require('js-yaml');

// Load configuration from YAML file
let config = {};
try {
  const configFile = fs.readFileSync(path.join(__dirname, 'config.yaml'), 'utf8');
  config = yaml.load(configFile);
  console.log('Loaded configuration:', config);
} catch (e) {
  console.error('Error loading configuration:', e);
  config = { 
    camera: { facingMode: "user" },
    audio: { mode: "wav", enabled: true }
  }; // Default config
}

const app = express();
let ttsClients = new Set();
let rosPublishers = {
  compressed: null,
  cameraInfo: null,
  pose: null,
  audio: null
};

// Initialize ROS2 node
async function initRos() {
  await rclnodejs.init();
  const node = rclnodejs.createNode('mobile_sensor_node');
  
  // Create publishers for camera data
  rosPublishers.compressed = node.createPublisher(
    'sensor_msgs/msg/CompressedImage',
    'camera/image_raw/compressed',
    { qos: { depth: 10 } }
  );

  rosPublishers.cameraInfo = node.createPublisher(
    'sensor_msgs/msg/CameraInfo',
    'camera/camera_info',
    { qos: { depth: 10 } }
  );

  // Add pose publisher
  rosPublishers.pose = node.createPublisher(
    'geometry_msgs/msg/Pose',
    'mobile_sensor/pose',
    { qos: { depth: 10 } }
  );

  // Add Audio publisher
  rosPublishers.audio = node.createPublisher(
    'std_msgs/msg/String',
    'mobile_sensor/speech',
    { qos: { depth: 10 } }
  );
  
  // Add string subscriber for TTS
  node.createSubscription(
    'std_msgs/msg/String',
    'mobile_sensor/tts',
    (msg) => {
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
    'tts_wav', // Subscribe to the topic used by audio_testing_node.py
    (msg) => {
      try {
        const buffer = Buffer.from(msg.data);
        
        // Check if the buffer has a WAV header (starts with 'RIFF' and contains 'WAVE')
        const hasWavHeader = 
          buffer.length >= 12 && 
          buffer[0] === 82 && buffer[1] === 73 && buffer[2] === 70 && buffer[3] === 70 && // 'RIFF'
          buffer[8] === 87 && buffer[9] === 65 && buffer[10] === 86 && buffer[11] === 69; // 'WAVE'
        
        // If header is present, log it and forward as-is
        // The client no longer needs to add a header
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
    'wav_bytes',
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

// Serve static files from the "public" folder
app.use(express.static('public'));
app.use(express.json());

// Serve index.html on GET /
app.get('/', (req, res) => {
  res.sendFile(path.join(__dirname, 'public', 'index.html'));
});

// Add an API endpoint to expose configuration
app.get('/api/config', (req, res) => {
  res.json(config);
});

const options = {
    key: fs.readFileSync(path.join(__dirname, 'key.pem')),
    cert: fs.readFileSync(path.join(__dirname, 'cert.pem')),
};

// Create HTTPS server
const server = https.createServer(options, app);

// Create WebSocket servers for different data types
const wssPose = new WebSocket.Server({ noServer: true });
const wssCamera = new WebSocket.Server({ noServer: true });
const wssTTS = new WebSocket.Server({ noServer: true });
const wssAudio = new WebSocket.Server({ noServer: true });
const wssWavAudio = new WebSocket.Server({ noServer: true }); // WebSocket server for WAV audio streaming

// Handle upgrade requests to separate WebSocket connections
server.on('upgrade', (request, socket, head) => {
  const pathname = new URL(request.url, `http://${request.headers.host}`).pathname;

  switch (pathname) {
    case '/tts':
      wssTTS.handleUpgrade(request, socket, head, (ws) => {
        wssTTS.emit('connection', ws, request);
      });
      break;
    case '/pose':
      wssPose.handleUpgrade(request, socket, head, (ws) => {
        wssPose.emit('connection', ws, request);
      });
      break;
    case '/camera':
      wssCamera.handleUpgrade(request, socket, head, (ws) => {
        wssCamera.emit('connection', ws, request);
      });
      break;
    case '/audio':
      wssAudio.handleUpgrade(request, socket, head, (ws) => {
        wssAudio.emit('connection', ws, request);
      });
      break;
    case '/wav_audio':
      wssWavAudio.handleUpgrade(request, socket, head, (ws) => {
        wssWavAudio.emit('connection', ws, request);
      });
      break;
    default:
      socket.destroy();
  }
});

// Handle pose data WebSocket connections
wssPose.on('connection', (ws) => {
  console.log('New pose data WebSocket client connected');
  ws.on('message', async (message) => {
    try {
      const data = JSON.parse(message);
      if (data.pose) {
        // Create and publish ROS2 pose message
        const poseMsg = {
          position: {
            x: data.pose.position.x,
            y: data.pose.position.y,
            z: data.pose.position.z
          },
          orientation: {
            x: data.pose.orientation.x,
            y: data.pose.orientation.y,
            z: data.pose.orientation.z,
            w: data.pose.orientation.w
          }
        };
        rosPublishers.pose.publish(poseMsg);
      }
    } catch (err) {
      console.error('Error processing pose message:', err);
    }
  });
  
  // Add disconnect logging
  ws.on('close', () => {
    console.log('Pose data WebSocket client disconnected');
  });
});

// Handle camera data WebSocket connections
wssCamera.on('connection', (ws) => {
  console.log('New camera data WebSocket client connected');
  ws.on('message', async (message) => {
    try {
      const data = JSON.parse(message);
      if (data.camera) {
        // Process camera frame
        // console.log('Received camera frame:', 
        //   `${data.camera.substring(0, 50)}... [${data.camera.length} chars]`);
        
        try {
          // Convert base64 string to binary data
          const base64Data = data.camera.split(',')[1]; // Remove data URL prefix if present
          const imageBuffer = Buffer.from(base64Data, 'base64');
          
          // Extract image dimensions if available, or use defaults
          const width = data.width || 640;
          const height = data.height || 480;
          
          // Generate current timestamp
          const now = Date.now();
          const stamp = {
            sec: Math.floor(now / 1000),
            nanosec: (now % 1000) * 1000000
          };
          
          // Create standard header
          const header = {
            stamp: stamp,
            frame_id: 'camera_frame'
          };
          
          // 1. Publish CompressedImage message (most efficient for JPEG data)
          const compressedMsg = {
            header: header,
            format: 'jpeg',
            data: Array.from(imageBuffer)
          };
          rosPublishers.compressed.publish(compressedMsg);
          
          // 2. Publish CameraInfo message
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
          rosPublishers.cameraInfo.publish(cameraInfoMsg);
          
          // console.log('Published camera data to ROS2 topics');
        } catch (error) {
          console.error('Error publishing to ROS2:', error);
        }
      }
    } catch (err) {
      console.error('Error processing camera message:', err);
    }
  });
  
  // Add disconnect logging
  ws.on('close', () => {
    console.log('Camera data WebSocket client disconnected');
  });
});

// Handle TTS WebSocket connections
wssTTS.on('connection', (ws) => {
  console.log('New TTS WebSocket client connected');
  ttsClients.add(ws);
  
  ws.on('close', () => {
    console.log('TTS WebSocket client disconnected');
    ttsClients.delete(ws);
  });
});

// Update audio WebSocket handler
wssAudio.on('connection', (ws) => {
  console.log('New audio WebSocket client connected');
  ws.on('message', async (message) => {
    try {
      const data = JSON.parse(message);
      if (data.transcription) {
        console.log(`Transcription received: "${data.transcription}"`);
        
        // Create timestamped message with header
        const now = Date.now();
        const msg = {
          header: {
            stamp: {
              sec: Math.floor(now / 1000),
              nanosec: (now % 1000) * 1000000
            },
            frame_id: 'audio_frame'
          },
          data: data.transcription
        };
        rosPublishers.audio.publish(msg);
      }
    } catch (err) {
      console.error('Error processing audio message:', err);
    }
  });
  
  // Add disconnect logging
  ws.on('close', () => {
    console.log('Audio WebSocket client disconnected');
  });
});

// Handle WAV audio streaming WebSocket connections
wssWavAudio.on('connection', (ws) => {
  console.log('New WAV audio streaming WebSocket client connected');
  
  ws.on('close', () => {
    console.log('WAV audio streaming WebSocket client disconnected');
  });
  
  // Send a connection confirmation message
  ws.send(JSON.stringify({ status: 'connected', message: 'Ready to receive audio data' }));
});

function shutdown() {
  console.log('Shutting down server...');
  
  wssPose.clients.forEach(client => client.close());
  wssCamera.clients.forEach(client => client.close());
  wssTTS.clients.forEach(client => client.close());
  wssAudio.clients.forEach(client => client.close());
  wssWavAudio.clients.forEach(client => client.close()); // Close WAV audio clients

  if (rclnodejs.isShutdown() === false) {
    rclnodejs.shutdown();
    console.log('ROS2 node shut down');
  }
  
  server.close(() => {
    console.log('HTTPS server closed');
    process.exit(0);
  });
}

// Listen for termination signals
process.on('SIGINT', shutdown);
process.on('SIGTERM', shutdown);
process.on('uncaughtException', (err) => {
  console.error('Uncaught exception:', err);
  shutdown();
});

// Initialize ROS2 and start server
initRos().then((node) => {
  const port = process.env.PORT || 4000;
  
  // Get network interfaces to show all available IP addresses
  const { networkInterfaces } = require('os');
  
  server.listen(port, '0.0.0.0', () => {
    // console.log(`HTTPS server running on port ${port}`);
    
    // Display all network interfaces for easy connection
    const nets = networkInterfaces();
    const results = {};
    
    for (const name of Object.keys(nets)) {
      for (const net of nets[name]) {
        // Skip over non-IPv4 and internal (loopback) addresses
        if (net.family === 'IPv4' && !net.internal) {
          if (!results[name]) {
            results[name] = [];
          }
          results[name].push(net.address);
          console.log(`Access URL: https://${net.address}:${port}`);
        }
      }
    }
    // Start ROS2 spinning
    rclnodejs.spin(node);
  });
}).catch((err) => {
  console.error('Failed to initialize ROS2:', err);
  process.exit(1);
});