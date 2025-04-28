/**
 * Express Server Configuration
 * Sets up Express app, routes, and middleware
 */
const express = require('express');
const path = require('path');
const https = require('https');
const fs = require('fs');

// Initialize Express application
function createExpressApp(config) {
  const app = express();
  
  // Serve static files from the "public" folder
  app.use(express.static(path.join(__dirname, 'public')));
  app.use(express.json());

  // Serve index.html on GET /
  app.get('/', (req, res) => {
    res.sendFile(path.join(__dirname, 'public', 'index.html'));
  });

  // Add an API endpoint to expose configuration
  app.get('/api/config', (req, res) => {
    res.json(config);
  });
  
  return app;
}

// Create HTTPS server
function createHttpsServer(app) {
  try {
    const options = {
      key: fs.readFileSync(path.join(__dirname, 'key.pem')),
      cert: fs.readFileSync(path.join(__dirname, 'cert.pem')),
    };
    
    return https.createServer(options, app);
  } catch (error) {
    console.error('Error creating HTTPS server:', error);
    throw error;
  }
}

// Start the server on the specified port
function startServer(server, port = 4000) {
  return new Promise((resolve, reject) => {
    try {
      server.listen(port, '0.0.0.0', () => {
        console.log(`HTTPS server running on port ${port}`);
        
        // Display all network interfaces for easy connection
        const { networkInterfaces } = require('os');
        const nets = networkInterfaces();
        
        for (const name of Object.keys(nets)) {
          for (const net of nets[name]) {
            // Skip over non-IPv4 and internal (loopback) addresses
            if (net.family === 'IPv4' && !net.internal) {
              console.log(`Access URL: https://${net.address}:${port}`);
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
  return new Promise((resolve, reject) => {
    if (!server) {
      resolve();
      return;
    }
    
    server.close(err => {
      if (err) {
        console.error('Error stopping server:', err);
        reject(err);
        return;
      }
      
      console.log('HTTPS server closed');
      resolve();
    });
  });
}

module.exports = {
  createExpressApp,
  createHttpsServer,
  startServer,
  stopServer
};