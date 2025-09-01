/**
 * Mobile Sensor Bridge - Main Application
 * 
 * This file serves as the entry point for the mobile sensor bridge application.
 * It loads configuration, initializes components, and manages the application lifecycle.
 */
const fs = require('fs');
const path = require('path');
const yaml = require('js-yaml');
const Logger = require('./logger');

// Import modular components
const rosInterface = require('./ros_interface');
const expressServer = require('./express_server');
const websocketHandlers = require('./websocket_handlers');

// Keep app startup header
Logger.drawHeader('MOBILE SENSOR BRIDGE');

// Load configuration from YAML file
let config = {};
try {
  const configFile = fs.readFileSync(path.join(__dirname, '../../config/config.yaml'), 'utf8');
  config = yaml.load(configFile);
  Logger.success('CONFIG', 'Configuration loaded successfully');
  
  // Set debug mode from config if available - handle new nested structure
  if (config.debug && config.debug.debug_logging !== undefined) {
    Logger.setDebugEnabled(config.debug.debug_logging === true);
  }
  
  // Set fancy logging mode from config if available - handle new nested structure
  if (config.debug && config.debug.color_logging !== undefined) {
    Logger.setFancyLoggingEnabled(config.debug.color_logging === true);
    Logger.info('CONFIG', `Fancy logging ${config.debug.color_logging ? 'enabled' : 'disabled'}`);
  }
} catch (e) {
  // Keep error logs for configuration issues
  Logger.error('CONFIG', `Error loading configuration: ${e.message}`);
  config = { 
    camera: { facingMode: "user" },
    audio: { mode: "wav", enabled: true }
  }; // Default config
  Logger.info('CONFIG', 'Using default configuration');
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
    // Keep ROS initialization success log
    Logger.success('ROS', 'ROS2 nodes initialized successfully');
    
    // Start the HTTPS server
    const port = process.env.PORT || 4000;
    await expressServer.startServer(server, port);
    
    // Start ROS2 spinning
    rosInterface.startSpinning();
    // Keep application startup success log
    Logger.success('APP', 'ROS sensor bridge activated successfully');
    
    return true;
  } catch (error) {
    // Keep error logs for startup issues
    Logger.error('APP', `Failed to start sensor bridge: ${error.message}`);
    await shutdown();
    return false;
  }
}

// Shutdown function to cleanly stop all components
async function shutdown() {
  // Keep shutdown header for visibility
  Logger.drawHeader('SHUTTING DOWN');
  
  try {
    // Close all WebSocket connections
    websocketHandlers.closeAllConnections();
    // Comment out less critical shutdown logs
    // Logger.info('WS', 'WebSocket connections closed');
    
    // Create a timeout promise to ensure we don't hang
    const timeoutPromise = new Promise(resolve => setTimeout(() => {
      // Keep timeout warning logs
      Logger.warn('SHUTDOWN', 'Shutdown taking too long, forcing exit');
      resolve();
    }, 3000)); // 3 seconds timeout
    
    // Shutdown ROS2 node with timeout
    await Promise.race([
      rosInterface.shutdown(),
      timeoutPromise
    ]);
    // Comment out less critical shutdown logs
    // Logger.info('ROS', 'ROS2 node shut down');
    
    // Stop HTTPS server with timeout
    await Promise.race([
      expressServer.stopServer(server),
      timeoutPromise
    ]);
    // Comment out less critical shutdown logs
    // Logger.info('SERVER', 'HTTPS server stopped');
    
    // Keep final shutdown success log
    Logger.success('SHUTDOWN', 'Application shutdown complete');
  } catch (err) {
    // Keep error logs during shutdown
    Logger.error('SHUTDOWN', `Error during shutdown: ${err.message}`);
  } finally {
    // Always exit even if there are errors
    process.exit(0);
  }
}

// Listen for termination signals
process.on('SIGINT', shutdown);
process.on('SIGTERM', shutdown);
process.on('uncaughtException', (err) => {
  // Keep uncaught exception logs - critical for debugging
  Logger.error('SYSTEM', `Uncaught exception: ${err.message}`);
  shutdown();
});

// Start the application
startApp().then(success => {
  if (!success) {
    // Keep startup failure logs
    Logger.error('APP', 'Application failed to start properly');
    process.exit(1);
  }
}).catch(err => {
  // Keep fatal error logs
  Logger.error('APP', `Fatal error during application startup: ${err.message}`);
  process.exit(1);
});