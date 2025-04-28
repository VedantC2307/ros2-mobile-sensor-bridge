const https = require('https');
const fs = require('fs');
const express = require('express');
const path = require('path');
const WebSocket = require('ws');
const rclnodejs = require('rclnodejs');
const yaml = require('js-yaml');
const { networkInterfaces } = require('os');

// ====================================
// ENHANCED LOGGING INTERFACE
// ====================================
const Logger = {
  COLORS: {
    reset: '\x1b[0m',
    bright: '\x1b[1m',
    dim: '\x1b[2m',
    
    // Foreground colors
    black: '\x1b[30m',
    red: '\x1b[31m',
    green: '\x1b[32m',
    yellow: '\x1b[33m',
    blue: '\x1b[34m',
    magenta: '\x1b[35m',
    cyan: '\x1b[36m',
    white: '\x1b[37m',
    
    // Background colors
    bgRed: '\x1b[41m',
    bgGreen: '\x1b[42m',
    bgYellow: '\x1b[43m',
    bgBlue: '\x1b[44m'
  },
  
  LEVELS: {
    DEBUG: { color: 'cyan' },
    INFO: { color: 'green' },
    WARN: { color: 'yellow' },
    ERROR: { color: 'red' },
    CRITICAL: { color: 'bgRed' },
    SUCCESS: { color: 'green' },
    SYSTEM: { color: 'magenta' },
    NETWORK: { color: 'blue' },
    ROS: { color: 'cyan' },
    WS: { color: 'yellow' }
  },
  
  // Track if debug logging is enabled (default is false)
  // Will be updated after loading config
  debugEnabled: false,
  
  // Method to update debug status from config
  updateDebugStatus(config) {
    if (config && config.debug_logging !== undefined) {
      this.debugEnabled = config.debug_logging === true;
      this.info('LOGGER', `Debug logging ${this.debugEnabled ? 'enabled' : 'disabled'} by config file`);
    }
  },
  
  formatMessage(level, module, message, data = null) {
    const timestamp = new Date().toISOString().replace('T', ' ').substr(0, 19);
    const levelConfig = this.LEVELS[level] || this.LEVELS.INFO;
    const color = this.COLORS[levelConfig.color];
    const resetColor = this.COLORS.reset;
    const brightColor = this.COLORS.bright;
    
    let formattedMsg = 
      `${this.COLORS.dim}[${timestamp}]${resetColor} ` +
      `${color}${brightColor}${level.padEnd(8)}${resetColor} ` +
      `${this.COLORS.yellow}[${module}]${resetColor} ` +
      `${message}`;
      
    if (data) {
      if (typeof data === 'object') {
        try {
          // For objects, summarize or truncate to avoid overwhelming logs
          let dataStr;
          if (data instanceof Buffer) {
            dataStr = `Buffer[${data.length} bytes]`;
          } else if (Array.isArray(data)) {
            dataStr = `Array[${data.length} items]`;
          } else {
            dataStr = JSON.stringify(data);
            if (dataStr.length > 100) {
              dataStr = dataStr.substring(0, 97) + '...';
            }
          }
          formattedMsg += ` ${this.COLORS.dim}${dataStr}${resetColor}`;
        } catch (e) {
          formattedMsg += ` ${this.COLORS.dim}[Object]${resetColor}`;
        }
      } else {
        formattedMsg += ` ${this.COLORS.dim}${data}${resetColor}`;
      }
    }
    
    return formattedMsg;
  },
  
  log(level, module, message, data = null) {
    // Skip DEBUG level messages if debug logging is not enabled
    if (level === 'DEBUG' && !this.debugEnabled) {
      return;
    }
    
    const formattedMessage = this.formatMessage(level, module, message, data);
    console.log(formattedMessage);
  },
  
  debug(module, message, data = null) {
    this.log('DEBUG', module, message, data);
  },
  
  info(module, message, data = null) {
    this.log('INFO', module, message, data);
  },
  
  warn(module, message, data = null) {
    this.log('WARN', module, message, data);
  },
  
  error(module, message, data = null) {
    this.log('ERROR', module, message, data);
  },
  
  critical(module, message, data = null) {
    this.log('CRITICAL', module, message, data);
  },
  
  success(module, message, data = null) {
    this.log('SUCCESS', module, message, data);
  },
  
  system(module, message, data = null) {
    this.log('SYSTEM', module, message, data);
  },
  
  network(module, message, data = null) {
    this.log('NETWORK', module, message, data);
  },
  
  ros(module, message, data = null) {
    this.log('ROS', module, message, data);
  },
  
  ws(module, message, data = null) {
    this.log('WS', module, message, data);
  },
  
  drawLine() {
    console.log(`${this.COLORS.dim}${'='.repeat(80)}${this.COLORS.reset}`);
  },
  
  drawHeader(title) {
    this.drawLine();
    console.log(`${this.COLORS.bright}${this.COLORS.cyan}${' '.repeat(Math.floor((80 - title.length) / 2))}${title}${this.COLORS.reset}`);
    this.drawLine();
  }
};

// Load configuration from YAML file
let config = {};
try {
  const configFile = fs.readFileSync(path.join(__dirname, 'config.yaml'), 'utf8');
  config = yaml.load(configFile);
  Logger.success('CONFIG', 'Successfully loaded configuration');
  Logger.debug('CONFIG', 'Configuration details', config);
} catch (e) {
  Logger.error('CONFIG', 'Error loading configuration', e.message);
  config = { 
    camera: { facingMode: "user" },
    audio: { mode: "wav", enabled: true }
  }; // Default config
  Logger.info('CONFIG', 'Using default configuration', config);
}

// Update logger debug status from config
Logger.updateDebugStatus(config);

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
  Logger.system('ROS', 'Initializing ROS2 node...');
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

  // Add Audio publisher - renamed from microphone to audio
  rosPublishers.audio = node.createPublisher(
    'std_msgs/msg/String',
    'mobile_sensor/speech',
    { qos: { depth: 10 } }
  );
  
  Logger.ros('PUBLISHER', 'Created ROS2 publishers for camera, pose and audio data');
  
  // Add string subscriber for TTS
  node.createSubscription(
    'std_msgs/msg/String',
    'mobile_sensor/tts',
    (msg) => {
      // Forward the message to all connected TTS clients
      let clientCount = 0;
      wssTTS.clients.forEach(client => {
        if (client.readyState === WebSocket.OPEN) {
          client.send(msg.data);
          clientCount++;
        }
      });
      if (clientCount > 0) {
        Logger.ros('SUBSCRIBER', `Received TTS message and forwarded to ${clientCount} clients`);
      }
    },
    { qos: { depth: 10 } }
  );
  
  // Update UInt8MultiArray subscriber for WAV audio
  node.createSubscription(
    'std_msgs/msg/UInt8MultiArray',
    'mobile_sensor/tts_wav',
    (msg) => {
      try {
        const buffer = Buffer.from(msg.data);
        
        let clientCount = 0;
        wssWavAudio.clients.forEach(client => {
          if (client.readyState === WebSocket.OPEN) {
            client.send(buffer);
            clientCount++;
          }
        });
        
        if (clientCount > 0) {
          Logger.ros('AUDIO', `Forwarded WAV audio: ${buffer.length} bytes to ${clientCount} clients`);
        }
      } catch (error) {
        Logger.error('AUDIO', 'Error processing audio data', error.message);
      }
    },
    { qos: { depth: 10 } }
  );
  
  // Keep the original wav_bytes subscription for backward compatibility
  node.createSubscription(
    'std_msgs/msg/UInt8MultiArray',
    'wav_bytes',
    (msg) => {
      try {
        const buffer = Buffer.from(msg.data);
        
        let clientCount = 0;
        wssWavAudio.clients.forEach(client => {
          if (client.readyState === WebSocket.OPEN) {
            client.send(buffer);
            clientCount++;
          }
        });
        
        // Only log when there are connected clients
        if (clientCount > 0 && buffer.length > 1000) {
          Logger.debug('AUDIO', `Legacy WAV audio sent: ${buffer.length} bytes`);
        }
      } catch (error) {
        Logger.error('AUDIO', 'Error processing legacy audio data', error.message);
      }
    },
    { qos: { depth: 10 } }
  );
  
  Logger.success('ROS', 'ROS2 node initialization complete');
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
const wssWavAudio = new WebSocket.Server({ noServer: true });

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
      Logger.warn('WS', `Rejected connection to unknown endpoint: ${pathname}`);
  }
});

// Handle pose data WebSocket connections
wssPose.on('connection', (ws) => {
  Logger.ws('POSE', 'New pose data WebSocket client connected');
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
        Logger.debug('POSE', 'Published pose data', {
          x: data.pose.position.x.toFixed(2),
          y: data.pose.position.y.toFixed(2),
          z: data.pose.position.z.toFixed(2)
        });
      }
    } catch (err) {
      Logger.error('POSE', 'Error processing pose message', err.message);
    }
  });
  
  ws.on('close', () => {
    Logger.ws('POSE', 'Pose data WebSocket client disconnected');
  });
});

// Handle camera data WebSocket connections
wssCamera.on('connection', (ws) => {
  Logger.ws('CAMERA', 'New camera data WebSocket client connected');
  // Add timestamp tracking for camera frame logging
  let lastCameraLogTime = 0;
  
  ws.on('message', async (message) => {
    try {
      const data = JSON.parse(message);
      if (data.camera) {
        try {
          // Convert base64 string to binary data
          const base64Data = data.camera.split(',')[1]; // Remove data URL prefix if present
          const imageBuffer = Buffer.from(base64Data, 'base64');
          
          // Extract image dimensions if available, or use defaults
          const width = data.width || 480;
          const height = data.height || 640;
          
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
          
          // 1. Publish CompressedImage message
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
          
          // Log camera frames at a reduced frequency to avoid log spam
          // Use a timestamp-based approach to only log once per second
          const currentSecond = Math.floor(now / 1000);
          if (currentSecond > lastCameraLogTime) {
            Logger.debug('CAMERA', `Published camera frame: ${width}x${height}, ${imageBuffer.length} bytes`);
            lastCameraLogTime = currentSecond;
          }
        } catch (error) {
          Logger.error('CAMERA', 'Error publishing to ROS2', error.message);
        }
      }
    } catch (err) {
      Logger.error('CAMERA', 'Error processing camera message', err.message);
    }
  });
  
  ws.on('close', () => {
    Logger.ws('CAMERA', 'Camera data WebSocket client disconnected');
  });
});

// Handle TTS WebSocket connections
wssTTS.on('connection', (ws) => {
  Logger.ws('TTS', 'New TTS WebSocket client connected');
  ttsClients.add(ws);
  
  // Add error handling
  ws.on('error', (error) => {
    Logger.error('TTS', 'WebSocket error:', error);
  });
  
  // Add message handling
  ws.on('message', (message) => {
    try {
      Logger.debug('TTS', 'Received message from client:', typeof message === 'string' ? message : `[Binary data: ${message.length} bytes]`);
    } catch (error) {
      Logger.error('TTS', 'Error processing client message:', error);
    }
  });
  
  ws.on('close', () => {
    Logger.ws('TTS', 'TTS WebSocket client disconnected');
    ttsClients.delete(ws);
  });
  
  // Send a connection confirmation message
  ws.send(JSON.stringify({ 
    status: 'connected', 
    message: 'TTS connection established',
    sessionState: 'active' // This matches the WAV audio format for consistency
  }));
  
  Logger.debug('TTS', 'Sent connection confirmation to client');
});

// Update audio WebSocket handler (not microphone - this receives speech transcription from web client)
wssAudio.on('connection', (ws) => {
  Logger.ws('AUDIO', 'New audio WebSocket client connected');
  ws.on('message', async (message) => {
    try {
      const data = JSON.parse(message);
      
      // Process transcription data from the web client's AudioManager
      if (data.transcription) {
        Logger.info('TRANSCRIPTION', `Speech received: "${data.transcription}"`);
        
        // Create timestamped message with header for ROS2
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
        
        // Publish to mobile_sensor/speech topic
        rosPublishers.audio.publish(msg);
        Logger.ros('AUDIO', 'Published speech transcription to ROS2');
      }
    } catch (err) {
      Logger.error('AUDIO', 'Error processing audio message', err.message);
    }
  });
  
  ws.on('close', () => {
    Logger.ws('AUDIO', 'Audio WebSocket client disconnected');
  });
});

// Handle WAV audio streaming WebSocket connections
wssWavAudio.on('connection', (ws) => {
  Logger.ws('WAV_AUDIO', 'New WAV audio streaming WebSocket client connected');
  
  // Track client state in the connection
  ws.isReady = true;
  
  ws.on('message', (message) => {
    try {
      Logger.debug('WAV_AUDIO', 'Received message from client:', typeof message === 'string' ? message : `[Binary data: ${message.length} bytes]`);
      // Handle client messages if needed
    } catch (error) {
      Logger.error('WAV_AUDIO', 'Error processing client message:', error);
    }
  });
  
  ws.on('close', () => {
    Logger.ws('WAV_AUDIO', 'WAV audio streaming WebSocket client disconnected');
  });
  
  ws.on('error', (error) => {
    Logger.error('WAV_AUDIO', 'WebSocket error:', error);
    ws.isReady = false;
  });
  
  // Send a connection confirmation message
  ws.send(JSON.stringify({ 
    status: 'connected', 
    message: 'Ready to receive audio data',
    sessionState: 'active' // This is important for the client to know it can play audio
  }));
  
  Logger.debug('WAV_AUDIO', 'Sent connection confirmation to client');
});

function shutdown() {
  Logger.drawHeader('SHUTTING DOWN SERVER');
  
  wssPose.clients.forEach(client => client.close());
  wssCamera.clients.forEach(client => client.close());
  wssTTS.clients.forEach(client => client.close());
  wssAudio.clients.forEach(client => client.close());
  wssWavAudio.clients.forEach(client => client.close());
  Logger.system('SHUTDOWN', 'Closed all WebSocket connections');

  if (rclnodejs.isShutdown() === false) {
    rclnodejs.shutdown();
    Logger.system('SHUTDOWN', 'ROS2 node shut down');
  }
  
  server.close(() => {
    Logger.system('SHUTDOWN', 'HTTPS server closed');
    Logger.drawLine();
    process.exit(0);
  });
}

// Listen for termination signals
process.on('SIGINT', shutdown);
process.on('SIGTERM', shutdown);
process.on('uncaughtException', (err) => {
  Logger.critical('SYSTEM', 'Uncaught exception', err.message);
  shutdown();
});

// Initialize ROS2 and start server
Logger.drawHeader('STARTING ROS2 ANDROID SENSOR INTERFACE');

initRos().then((node) => {
  const port = process.env.PORT || 4000;
  
  server.listen(port, '0.0.0.0', () => {
    Logger.success('SERVER', `HTTPS server running on port ${port}`);
    
    // Display all network interfaces for easy connection
    const nets = networkInterfaces();
    const results = {};
    
    Logger.drawHeader('ACCESS URLS');
    
    for (const name of Object.keys(nets)) {
      for (const net of nets[name]) {
        // Skip over non-IPv4 and internal (loopback) addresses
        if (net.family === 'IPv4' && !net.internal) {
          if (!results[name]) {
            results[name] = [];
          }
          results[name].push(net.address);
          Logger.network('URL', `https://${net.address}:${port}`);
        }
      }
    }
    
    Logger.drawLine();
    
    // Start ROS2 spinning
    rclnodejs.spin(node);
    Logger.ros('SPIN', 'Started ROS2 node spinning');
  });
}).catch((err) => {
  Logger.critical('INIT', 'Failed to initialize ROS2', err.message);
  process.exit(1);
});