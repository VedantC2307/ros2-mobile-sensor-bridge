/**
 * WebSocket Handlers for the mobile sensor bridge
 * Manages WebSocket connections and event handling for different data types
 */
const WebSocket = require('ws');
const rosInterface = require('./ros_interface');
const Logger = require('./logger');

// Store WebSocket servers for use across the module
let servers = {
  pose: null,
  camera: null,
  tts: null,
  microphone: null, // Changed from audio to microphone for clarity
  wavAudio: null,
  imu: null, // Added for iOS IMU sensor data
  gps: null  // Added for GPS location data
};

// Track TTS clients
let ttsClients = new Set();

// Initialize all WebSocket servers and attach them to the HTTP server
function initWebSockets(server) {
  // Create WebSocket servers for different data types
  servers.pose = new WebSocket.Server({ noServer: true });
  servers.camera = new WebSocket.Server({ noServer: true });
  servers.tts = new WebSocket.Server({ noServer: true });
  servers.microphone = new WebSocket.Server({ noServer: true }); // Changed from audio to microphone
  servers.wavAudio = new WebSocket.Server({ noServer: true });
  servers.imu = new WebSocket.Server({ noServer: true }); // Added for iOS and Android IMU sensor data
  servers.gps = new WebSocket.Server({ noServer: true }); // Added for GPS location data
  
  // Set up WebSocket route handlers
  server.on('upgrade', (request, socket, head) => {
    const pathname = new URL(request.url, `http://${request.headers.host}`).pathname;

    switch (pathname) {
      case '/tts':
        servers.tts.handleUpgrade(request, socket, head, (ws) => {
          servers.tts.emit('connection', ws, request);
        });
        break;
      case '/pose':
        servers.pose.handleUpgrade(request, socket, head, (ws) => {
          servers.pose.emit('connection', ws, request);
        });
        break;
      case '/camera':
        servers.camera.handleUpgrade(request, socket, head, (ws) => {
          servers.camera.emit('connection', ws, request);
        });
        break;
      case '/imu':
        servers.imu.handleUpgrade(request, socket, head, (ws) => {
          servers.imu.emit('connection', ws, request);
        });
        break;
      case '/gps':
        servers.gps.handleUpgrade(request, socket, head, (ws) => {
          servers.gps.emit('connection', ws, request);
        });
        break;
      case '/microphone': // Changed from /audio to /microphone
        servers.microphone.handleUpgrade(request, socket, head, (ws) => {
          servers.microphone.emit('connection', ws, request);
        });
        break;
      case '/wav_audio':
        servers.wavAudio.handleUpgrade(request, socket, head, (ws) => {
          servers.wavAudio.emit('connection', ws, request);
        });
        break;
      default:
        socket.destroy();
    }
  });
  
  // Initialize event handlers for each WebSocket type
  setupPoseHandlers();
  setupCameraHandlers();
  setupTTSHandlers();
  setupMicrophoneHandlers(); // Changed from setupAudioHandlers
  setupWavAudioHandlers();
  setupIMUHandlers(); // Added for iOS and Android IMU sensor data
  setupGPSHandlers(); // Added for GPS location data
  
  return servers;
}

// Set up pose data WebSocket handlers
function setupPoseHandlers() {
  servers.pose.on('connection', (ws) => {
    Logger.info('APP', 'Pose data sensor activated');
    ws.on('message', async (message) => {
      try {
        const data = JSON.parse(message);
        if (data.pose) {
          // Use ROS interface to publish pose data
          rosInterface.publishPoseData(data.pose, {
            sec: Math.floor(data.timestamp / 1000),
            nanosec: (data.timestamp % 1000) * 1000000
          });
        }
      } catch (err) {
        Logger.error('ROS', `Error processing pose message: ${err}`);
      }
    });
    
    // Add disconnect logging
    ws.on('close', () => {
      Logger.info('APP', 'Pose data sensor deactivated');
    });
  });
}

// Set up camera data WebSocket handlers
function setupCameraHandlers() {
  servers.camera.on('connection', (ws) => {
    Logger.info('APP', 'Camera sensor activated');
    ws.on('message', async (message) => {
      try {
        const data = JSON.parse(message);
        if (data.camera) {
          try {
            // Convert base64 string to binary data
            const base64Data = data.camera.split(',')[1]; // Remove data URL prefix if present
            const imageBuffer = Buffer.from(base64Data, 'base64');
            
            // Extract image dimensions if available, or use defaults
            const width = data.width || 640;
            const height = data.height || 480;
            
            // Generate timestamp from data or current time
            const stamp = {
              sec: Math.floor(data.timestamp ? data.timestamp / 1000 : Date.now() / 1000),
              nanosec: (data.timestamp ? data.timestamp % 1000 : Date.now() % 1000) * 1000000
            };
            
            // Use ROS interface to publish camera data
            rosInterface.publishCameraData(imageBuffer, width, height, stamp);
          } catch (error) {
            Logger.error('ROS', `Error publishing camera data to ROS2: ${error}`);
          }
        }
      } catch (err) {
        Logger.error('ROS', `Error processing camera message: ${err}`);
      }
    });
    
    // Add disconnect logging
    ws.on('close', () => {
      Logger.info('APP', 'Camera sensor deactivated');
    });
  });
}

// Set up TTS WebSocket handlers
function setupTTSHandlers() {
  servers.tts.on('connection', (ws) => {
    Logger.info('APP', 'Text-to-speech node activated');
    
    ttsClients.add(ws);
    
    // Send a welcome message to verify the connection works
    try {
      ws.send("TTS system ready");
      Logger.info('ROS', 'TTS node initialized');
    } catch (error) {
      Logger.error('ROS', `Error initializing TTS node: ${error}`);
    }
    
    ws.on('close', () => {
      Logger.info('APP', 'Text-to-speech node deactivated');
      ttsClients.delete(ws);
    });
    
    ws.on('error', (error) => {
      Logger.error('ROS', `TTS node error: ${error}`);
    });
  });
}

// Set up microphone WebSocket handlers (renamed from setupAudioHandlers)
function setupMicrophoneHandlers() {
  servers.microphone.on('connection', (ws) => {
    Logger.info('APP', 'Microphone sensor activated');
    
    ws.on('message', async (message) => {
      try {
        const data = JSON.parse(message);
        if (data.transcription) {
          Logger.info('ROS', `Transcription received: "${data.transcription}"`);
          
          // Use ROS interface to publish microphone transcription
          rosInterface.publishMicrophoneTranscription(data.transcription, 
            data.header && data.header.stamp ? data.header.stamp : null);
        }
      } catch (err) {
        Logger.error('ROS', `Error processing microphone message: ${err}`);
      }
    });
    
    // Add disconnect logging
    ws.on('close', () => {
      Logger.info('APP', 'Microphone sensor deactivated');
    });
  });
}

// Set up WAV audio streaming WebSocket handlers
function setupWavAudioHandlers() {
  servers.wavAudio.on('connection', (ws) => {
    Logger.info('APP','Audio playback node activated');
    
    // Track client state in the connection
    ws.isReady = true;
    
    ws.on('message', (message) => {
      try {
        // Handle received messages if needed
      } catch (error) {
        Logger.error('ROS', `Error processing audio data: ${error}`);
      }
    });
    
    ws.on('close', () => {
      Logger.info('APP', 'Audio playback node deactivated');
    });
    
    ws.on('error', (error) => {
      Logger.error('ROS', `Audio data processing error: ${error}`);
      ws.isReady = false;
    });
    
    // Send a connection confirmation message
    ws.send(JSON.stringify({ 
      status: 'connected', 
      message: 'Ready to receive audio data',
      sessionState: 'active' // This is important for the client to know it can play audio
    }));
  });
}

// Close all WebSocket connections
function closeAllConnections() {
  Object.values(servers).forEach(server => {
    if (server && server.clients) {
      server.clients.forEach(client => {
        if (client.readyState === WebSocket.OPEN) {
          client.close();
        }
      });
    }
  });
  
  Logger.info('APP', 'All ROS sensor nodes deactivated');
}

// Set up IMU data WebSocket handlers for iOS and Android devices
function setupIMUHandlers() {
  servers.imu.on('connection', (ws) => {
    Logger.info('APP', 'IMU sensor activated');
    
    ws.on('message', async (message) => {
      try {
        const data = JSON.parse(message);
        if (data.imu) {
          // Log the IMU data to the console for debugging
          Logger.debug('IMU', `Accelerometer: x=${data.imu.accelerometer.x.toFixed(2)}, y=${data.imu.accelerometer.y.toFixed(2)}, z=${data.imu.accelerometer.z.toFixed(2)}`);
          Logger.debug('IMU', `Gyroscope: alpha=${data.imu.gyroscope.alpha.toFixed(2)}, beta=${data.imu.gyroscope.beta.toFixed(2)}, gamma=${data.imu.gyroscope.gamma.toFixed(2)}`);
          
          // Generate timestamp from IMU data or current time
          const stamp = {
            sec: Math.floor(data.imu.timestamp ? data.imu.timestamp / 1000 : Date.now() / 1000),
            nanosec: (data.imu.timestamp ? data.imu.timestamp % 1000 : Date.now() % 1000) * 1000000
          };
          
          // Use ROS interface to publish IMU data
          rosInterface.publishIMUData(data.imu, stamp);
        }
      } catch (err) {
        Logger.error('ROS', `Error processing IMU message: ${err}`);
      }
    });
    
    ws.on('close', () => {
      Logger.info('APP', 'IMU sensor deactivated');
    });
  });
}

// Set up GPS data WebSocket handlers for iOS and Android devices
function setupGPSHandlers() {
  servers.gps.on('connection', (ws) => {
    Logger.info('APP', 'GPS sensor activated');
    
    ws.on('message', async (message) => {
      try {
        const data = JSON.parse(message);
        if (data.gps) {
          // Log the GPS data to the console for debugging
          Logger.debug('GPS', `Location: lat=${data.gps.latitude.toFixed(6)}, long=${data.gps.longitude.toFixed(6)}, alt=${data.gps.altitude.toFixed(2)}`);
          if (data.gps.accuracy) {
            Logger.debug('GPS', `Accuracy: ${data.gps.accuracy.toFixed(2)}m, Heading: ${data.gps.heading?.toFixed(2) || 'N/A'}, Speed: ${data.gps.speed?.toFixed(2) || 'N/A'}m/s`);
          }
          
          // Generate timestamp from GPS data or current time
          const stamp = {
            sec: Math.floor(data.gps.timestamp ? data.gps.timestamp / 1000 : Date.now() / 1000),
            nanosec: (data.gps.timestamp ? data.gps.timestamp % 1000 : Date.now() % 1000) * 1000000
          };
          
          // Use ROS interface to publish GPS data
          rosInterface.publishGPSData(data.gps, stamp);
        }
      } catch (err) {
        Logger.error('ROS', `Error processing GPS message: ${err}`);
      }
    });
    
    ws.on('close', () => {
      Logger.info('APP', 'GPS sensor deactivated');
    });
  });
}

module.exports = {
  initWebSockets,
  closeAllConnections,
  getServers: () => servers,
  getTTSClients: () => ttsClients
};