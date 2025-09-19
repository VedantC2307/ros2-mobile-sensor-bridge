/**
 * Express Server Configuration
 * Sets up Express app, routes, and middleware
 */
const express = require('express');
const path = require('path');
const https = require('https');
const fs = require('fs');
const os = require('os');
const Logger = require('./logger'); // Import Logger module with correct capitalization

// Initialize Express application
function createExpressApp(config) {
  const app = express();
  
  // Serve static files from the "client" folder
  app.use(express.static(path.join(__dirname, '../client')));
  app.use(express.json());

  // Serve index.html on GET /
  app.get('/', (req, res) => {
    res.sendFile(path.join(__dirname, '../client', 'index.html'));
  });

  // Add an API endpoint to expose configuration
  app.get('/api/config', (req, res) => {
    // Prepare a safe version of the config to send to the client
    const clientConfig = {
      camera: config.camera || {},
      audio: config.audio || {},
      microphone: config.microphone || {},
      imu: config.imu || { sample_rate: 30 }, // Add IMU configuration
      debug: {
        // Handle nested debug properties
        'mobile-debug-console': config.debug && config.debug['mobile-debug-console'] || false
      }
    };
    
    res.json(clientConfig);
  });
  
  return app;
}

// Create HTTPS server
function createHttpsServer(app) {
  try {
    const options = {
      key: fs.readFileSync(path.join(__dirname, '../ssl/key.pem')),
      cert: fs.readFileSync(path.join(__dirname, '../ssl/cert.pem')),
    };
    
    return https.createServer(options, app);
  } catch (error) {
    Logger.error('SERVER', `Error creating HTTPS server: ${error}`);
    throw error;
  }
}

// Get local IP address for display
function getLocalIP() {
  const { networkInterfaces } = os;
  const nets = networkInterfaces();
  
  for (const name of Object.keys(nets)) {
    for (const net of nets[name]) {
      // Skip over non-IPv4 and internal (loopback) addresses
      if (net.family === 'IPv4' && !net.internal) {
        return net.address;
      }
    }
  }
  
  return 'localhost'; // Fallback if no suitable IP is found
}

// Start the server on the specified port
function startServer(server, port = 4000) {
  return new Promise((resolve, reject) => {
    try {
      server.listen(port, '0.0.0.0', () => {
        Logger.info('SERVER', `HTTPS server running on port ${port}`);
        
        // Display all network interfaces for easy connection
        const { networkInterfaces } = require('os');
        const nets = networkInterfaces();
        
        for (const name of Object.keys(nets)) {
          for (const net of nets[name]) {
            // Skip over non-IPv4 and internal (loopback) addresses
            if (net.family === 'IPv4' && !net.internal) {
              Logger.info('SERVER', `Access URL: https://${net.address}:${port}`);
            }
          }
        }
        
        resolve(server);
      });
    } catch (err) {
      reject(err);
    }
  });
}

// Stop the server
function stopServer(server) {
  return new Promise((resolve) => {
    if (!server) {
      Logger.info('SERVER', 'No server to stop');
      resolve();
      return;
    }
    
    // Set a timeout to ensure we don't hang
    const timeout = setTimeout(() => {
      Logger.warn('SERVER', 'Server close operation timed out, forcing resolution');
      resolve();
    }, 2000); // 2 second timeout
    
    try {
      server.close(err => {
        clearTimeout(timeout); // Clear the timeout as we got a response
        
        if (err) {
          Logger.error('SERVER', `Error stopping server: ${err}`);
        } else {
          Logger.success('SERVER', 'HTTPS server closed successfully');
        }
        
        // Always resolve even if there was an error
        resolve();
      });
    } catch (error) {
      clearTimeout(timeout);
      Logger.error('SERVER', `Exception stopping server: ${error}`);
      resolve(); // Still resolve to avoid hanging the shutdown process
    }
  });
}

module.exports = {
  createExpressApp,
  createHttpsServer,
  startServer,
  stopServer,
  getLocalIP
};