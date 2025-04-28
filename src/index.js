/**
 * Mobile Sensor Bridge - Main Application
 * 
 * This file serves as the entry point for the mobile sensor bridge application.
 * It loads configuration, initializes components, and manages the application lifecycle.
 */
const fs = require('fs');
const path = require('path');
const yaml = require('js-yaml');

// Import modular components
const rosInterface = require('./ros_interface');
const expressServer = require('./express_server');
const websocketHandlers = require('./websocket_handlers');

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

// Initialize Express application
const app = expressServer.createExpressApp(config);

// Create HTTPS server
const server = expressServer.createHttpsServer(app);

// Initialize WebSocket handlers
const wsServers = websocketHandlers.initWebSockets(server);

// Initialize and start the application
async function startApp() {
  try {
    // Initialize ROS2 node
    await rosInterface.initRos(wsServers.tts, wsServers.wavAudio);
    console.log('ROS2 node initialized');
    
    // Start the HTTPS server
    const port = process.env.PORT || 4000;
    await expressServer.startServer(server, port);
    
    // Start ROS2 spinning
    rosInterface.startSpinning();
    
    return true;
  } catch (error) {
    console.error('Failed to start application:', error);
    await shutdown();
    return false;
  }
}

// Shutdown function to cleanly stop all components
async function shutdown() {
  console.log('Shutting down server...');
  
  // Close all WebSocket connections
  websocketHandlers.closeAllConnections();
  
  // Shutdown ROS2 node
  rosInterface.shutdown();
  
  // Stop HTTPS server
  await expressServer.stopServer(server);
  
  console.log('Application shutdown complete');
  process.exit(0);
}

// Listen for termination signals
process.on('SIGINT', shutdown);
process.on('SIGTERM', shutdown);
process.on('uncaughtException', (err) => {
  console.error('Uncaught exception:', err);
  shutdown();
});

// Start the application
startApp().then(success => {
  if (!success) {
    console.error('Application failed to start properly');
    process.exit(1);
  }
}).catch(err => {
  console.error('Fatal error during application startup:', err);
  process.exit(1);
});