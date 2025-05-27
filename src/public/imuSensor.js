/**
 * IMU Sensor Manager
 * Handles device orientation and motion data from the device's inertial measurement unit (IMU)
 * Specialized for iOS devices
 */

class IMUSensorManager {
  constructor() {
    this.isActive = false;
    this.ws = null;
    this.sampleRate = 60; // Hz
    this.intervalId = null;
    
    // Store sensor data
    this.accelerometerData = { x: 0, y: 0, z: 0 };
    this.gyroscopeData = { alpha: 0, beta: 0, gamma: 0 };
    this.magnetometerData = { x: 0, y: 0, z: 0 };
    
    // Check if device is iOS
    this.isIOS = /iPad|iPhone|iPod/.test(navigator.userAgent) && !window.MSStream;
    
    // Device motion and orientation permission status
    this.permissionGranted = false;
  }

  /**
   * Request permission to access device motion and orientation on iOS
   * Required for iOS 13+ due to privacy restrictions
   * @returns {Promise} - Resolves when permission is granted, rejects when denied
   */
  async requestPermission() {
    if (!this.isIOS) {
      console.log('Not an iOS device, no need to request permission');
      return Promise.resolve(true);
    }
    
    // Check for DeviceMotionEvent API first
    const hasMotionAPI = typeof DeviceMotionEvent !== 'undefined' && 
                         typeof DeviceMotionEvent.requestPermission === 'function';
                         
    // Check for DeviceOrientationEvent API second
    const hasOrientationAPI = typeof DeviceOrientationEvent !== 'undefined' && 
                              typeof DeviceOrientationEvent.requestPermission === 'function';
    
    // iOS 13+ requires explicit permission
    if (hasMotionAPI || hasOrientationAPI) {
      try {
        console.log('Requesting sensor permissions for iOS device...');
        
        let motionState = 'granted'; // Default if API is not available
        let orientationState = 'granted'; // Default if API is not available
        
        // Request permission for DeviceMotionEvent if available
        if (hasMotionAPI) {
          console.log('Requesting DeviceMotionEvent permission...');
          motionState = await DeviceMotionEvent.requestPermission();
          console.log('Motion permission state:', motionState);
        }
        
        // Request permission for DeviceOrientationEvent if available
        if (hasOrientationAPI) {
          console.log('Requesting DeviceOrientationEvent permission...');
          orientationState = await DeviceOrientationEvent.requestPermission();
          console.log('Orientation permission state:', orientationState);
        }
        
        if (motionState === 'granted' && orientationState === 'granted') {
          console.log('Motion and orientation permissions granted');
          this.permissionGranted = true;
          return true;
        } else {
          console.warn('Motion or orientation permission denied');
          console.warn('Motion state:', motionState, 'Orientation state:', orientationState);
          return false;
        }
      } catch (error) {
        console.error('Error requesting sensor permissions:', error);
        console.error('Error details:', error.message);
        return false;
      }
    } else {
      // For non-iOS 13+ devices or desktop browsers
      console.log('Permission API not available, assuming granted');
      this.permissionGranted = true;
      return true;
    }
  }

  // Start IMU data collection and transmission
  async startIMUSensor(websocket, isSessionActive) {
    this.ws = websocket;
    this.isActive = isSessionActive;
    
    if (!this.isIOS) {
      console.log('IMU sensor only implemented for iOS devices');
      return false;
    }
    
    try {
      // Request permission if needed
      console.log('Starting IMU sensor and requesting permissions...');
      const permissionGranted = await this.requestPermission();
      
      if (!permissionGranted) {
        console.error('IMU sensor permission denied');
        // Show a user-friendly alert
        if (typeof alert === 'function') {
          setTimeout(() => {
            alert('IMU sensor access was denied. Please enable motion and orientation access in your device settings to use this feature.');
          }, 500);
        }
        return false;
      }
      
      console.log('Permission granted, setting up sensors...');
      
      // Setup event handlers for sensors
      this.setupAccelerometerAndGyroscope();
      this.setupMagnetometer();
      
      // Start sending data at the specified sample rate
      this.intervalId = setInterval(() => {
        this.sendSensorData();
      }, 1000 / this.sampleRate);
      
      console.log('iOS IMU sensor manager initialized successfully');
      return true;
    } catch (error) {
      console.error('Error initializing IMU sensor:', error);
      return false;
    }
  }

  // Set up accelerometer and gyroscope listeners
  setupAccelerometerAndGyroscope() {
    window.addEventListener('devicemotion', (event) => {
      if (!this.isActive) return;
      
      // Get accelerometer data (in m/s²)
      if (event.acceleration) {
        this.accelerometerData = {
          x: event.acceleration.x || 0,
          y: event.acceleration.y || 0,
          z: event.acceleration.z || 0
        };
      }
      
      // Get gyroscope data (in rad/s)
      if (event.rotationRate) {
        this.gyroscopeData = {
          alpha: event.rotationRate.alpha || 0, // rotation around z-axis
          beta: event.rotationRate.beta || 0,   // rotation around x-axis
          gamma: event.rotationRate.gamma || 0  // rotation around y-axis
        };
      }
    });
  }

  // Set up magnetometer listeners
  setupMagnetometer() {
    window.addEventListener('deviceorientation', (event) => {
      if (!this.isActive) return;
      
      // Get compass heading from device orientation
      // Note: This is a simplification as iOS doesn't expose raw magnetometer data directly
      // alpha: compass direction (degrees), beta: front-to-back tilt (degrees), gamma: left-to-right tilt (degrees)
      const alpha = event.alpha || 0;  // compass direction
      const beta = event.beta || 0;    // front/back tilt
      const gamma = event.gamma || 0;  // left/right tilt
      
      // We'll use the orientation values as our "magnetometer" data
      this.magnetometerData = {
        x: gamma,
        y: beta,
        z: alpha
      };
    });
  }

  // Send collected sensor data through WebSocket
  sendSensorData() {
    if (!this.isActive || !this.ws || this.ws.readyState !== WebSocket.OPEN) {
      return;
    }
    
    const timestamp = Date.now();
    
    // Create a structured payload
    const payload = {
      imu: {
        timestamp: timestamp,
        accelerometer: this.accelerometerData,
        gyroscope: this.gyroscopeData,
        magnetometer: this.magnetometerData
      }
    };
    
    // Send as JSON
    try {
      this.ws.send(JSON.stringify(payload));
    } catch (error) {
      console.error('Error sending IMU data:', error);
    }
  }

  // Stop IMU data collection
  stopIMUSensor() {
    this.isActive = false;
    
    if (this.intervalId) {
      clearInterval(this.intervalId);
      this.intervalId = null;
    }
    
    console.log('IMU sensor stopped');
    return Promise.resolve();
  }
}

// Make the IMU manager available globally
window.IMUSensorManager = IMUSensorManager;
