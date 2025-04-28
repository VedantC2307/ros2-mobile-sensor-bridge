/**
 * WebSocket Handlers for the mobile sensor bridge
 * Manages WebSocket connections and event handling for different data types
 */
const WebSocket = require('ws');
const rosInterface = require('./ros_interface');

// Store WebSocket servers for use across the module
let servers = {
  pose: null,
  camera: null,
  tts: null,
  microphone: null, // Changed from audio to microphone for clarity
  wavAudio: null
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
  
  return servers;
}

// Set up pose data WebSocket handlers
function setupPoseHandlers() {
  servers.pose.on('connection', (ws) => {
    console.log('New pose data WebSocket client connected');
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
        console.error('Error processing pose message:', err);
      }
    });
    
    // Add disconnect logging
    ws.on('close', () => {
      console.log('Pose data WebSocket client disconnected');
    });
  });
}

// Set up camera data WebSocket handlers
function setupCameraHandlers() {
  servers.camera.on('connection', (ws) => {
    console.log('New camera data WebSocket client connected');
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
            console.error('Error publishing camera data to ROS2:', error);
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
}

// Set up TTS WebSocket handlers
function setupTTSHandlers() {
  servers.tts.on('connection', (ws) => {
    console.log('New TTS WebSocket client connected');
    ttsClients.add(ws);
    
    // Send a welcome message to verify the connection works
    try {
      ws.send("TTS system ready");
      console.log('Sent welcome message to TTS client');
    } catch (error) {
      console.error('Error sending welcome message to TTS client:', error);
    }
    
    ws.on('close', () => {
      console.log('TTS WebSocket client disconnected');
      ttsClients.delete(ws);
    });
    
    ws.on('error', (error) => {
      console.error('TTS WebSocket error:', error);
    });
  });
}

// Set up microphone WebSocket handlers (renamed from setupAudioHandlers)
function setupMicrophoneHandlers() {
  servers.microphone.on('connection', (ws) => {
    console.log('New microphone WebSocket client connected');
    ws.on('message', async (message) => {
      try {
        const data = JSON.parse(message);
        if (data.transcription) {
          console.log(`Transcription received: "${data.transcription}"`);
          
          // Use ROS interface to publish microphone transcription
          rosInterface.publishMicrophoneTranscription(data.transcription, 
            data.header && data.header.stamp ? data.header.stamp : null);
        }
      } catch (err) {
        console.error('Error processing microphone message:', err);
      }
    });
    
    // Add disconnect logging
    ws.on('close', () => {
      console.log('Microphone WebSocket client disconnected');
    });
  });
}

// Set up WAV audio streaming WebSocket handlers
function setupWavAudioHandlers() {
  servers.wavAudio.on('connection', (ws) => {
    console.log('New WAV audio streaming WebSocket client connected');
    
    // Track client state in the connection
    ws.isReady = true;
    
    ws.on('message', (message) => {
      try {
        console.log('Received message from WAV audio client:', typeof message === 'string' ? message : `[Binary data: ${message.length} bytes]`);
        // Handle client messages if needed
      } catch (error) {
        console.error('Error processing WAV audio client message:', error);
      }
    });
    
    ws.on('close', () => {
      console.log('WAV audio streaming WebSocket client disconnected');
    });
    
    ws.on('error', (error) => {
      console.error('WAV audio WebSocket error:', error);
      ws.isReady = false;
    });
    
    // Send a connection confirmation message
    ws.send(JSON.stringify({ 
      status: 'connected', 
      message: 'Ready to receive audio data',
      sessionState: 'active' // This is important for the client to know it can play audio
    }));

    console.log('Sent connection confirmation to WAV audio client');
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
  
  console.log('All WebSocket connections closed');
}

module.exports = {
  initWebSockets,
  closeAllConnections,
  getServers: () => servers,
  getTTSClients: () => ttsClients
};