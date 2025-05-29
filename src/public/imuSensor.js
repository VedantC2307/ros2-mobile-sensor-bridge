/**
 * IMU Sensor Manager
 * Handles device motion data from the device's inertial measurement unit (IMU)
 * Supports both iOS and Android devices
 * Collects only accelerometer and gyroscope data
 */

class IMUSensorManager {
  constructor() {
    this.isActive = false;
    this.ws = null;
    this.sampleRate = 60; // Hz
    this.intervalId = null;
    
    // Store sensor data - only accelerometer and gyroscope
    this.accelerometerData = { x: 0, y: 0, z: 0 };
    this.gyroscopeData = { alpha: 0, beta: 0, gamma: 0 };
    
    // Device detection
    this.isIOS = /iPad|iPhone|iPod/.test(navigator.userAgent) && !window.MSStream;
    this.isAndroid = /Android/.test(navigator.userAgent);
    
    // Device motion permission status
    this.permissionGranted = false;
  }

  /**
   * Request permission to access device motion data
   * Required for iOS 13+ and modern Android browsers due to privacy restrictions
   * @returns {Promise} - Resolves when permission is granted, rejects when denied
   */
  async requestPermission() {
    // For Android devices - check if permission is needed
    if (this.isAndroid) {
      console.log('Android device detected, checking if permission is needed...');
      
      // Check if DeviceMotionEvent requires permission (modern Android browsers)
      const hasMotionAPI = typeof DeviceMotionEvent !== 'undefined' && 
                           typeof DeviceMotionEvent.requestPermission === 'function';
      
      if (hasMotionAPI) {
        try {
          console.log('Requesting DeviceMotionEvent permission for Android...');
          const motionState = await DeviceMotionEvent.requestPermission();
          console.log('Android motion permission state:', motionState);
          
          if (motionState === 'granted') {
            console.log('Android motion permission granted');
            this.permissionGranted = true;
            return true;
          } else {
            console.warn('Android motion permission denied:', motionState);
            return false;
          }
        } catch (error) {
          console.error('Error requesting Android sensor permissions:', error);
          return false;
        }
      } else {
        console.log('Android device: no explicit permission API available, assuming granted');
        this.permissionGranted = true;
        return Promise.resolve(true);
      }
    }
    
    // For non-iOS and non-Android devices
    if (!this.isIOS) {
      console.log('Not an iOS/Android device, no need to request permission');
      return Promise.resolve(true);
    }
    
    // iOS-specific permission handling
    // Check for DeviceMotionEvent API
    const hasMotionAPI = typeof DeviceMotionEvent !== 'undefined' && 
                         typeof DeviceMotionEvent.requestPermission === 'function';
    
    // iOS 13+ requires explicit permission
    if (hasMotionAPI) {
      try {
        console.log('Requesting motion permission for iOS device...');
        
        // Request permission for DeviceMotionEvent
        console.log('Requesting DeviceMotionEvent permission...');
        const motionState = await DeviceMotionEvent.requestPermission();
        console.log('Motion permission state:', motionState);
        
        if (motionState === 'granted') {
          console.log('Motion permission granted');
          this.permissionGranted = true;
          return true;
        } else {
          console.warn('Motion permission denied:', motionState);
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
    
    if (!this.isIOS && !this.isAndroid) {
      console.log('IMU sensor only implemented for iOS and Android devices');
      return false;
    }
    
    try {
      // Check Android sensor availability first if on Android
      if (this.isAndroid) {
        const sensorsAvailable = this.checkAndroidSensorAvailability();
        if (!sensorsAvailable) {
          console.error('Required sensors not available on this Android device');
          if (typeof alert === 'function') {
            setTimeout(() => {
              alert('This device does not support the required motion sensors. Some features may not work correctly.');
            }, 500);
          }
          // We don't return false here as we can still try with whatever sensors are available
        }
      }
      
      // Request permission if needed
      console.log('Starting IMU sensor and requesting permissions...');
      const permissionGranted = await this.requestPermission();
      
      if (!permissionGranted) {
        console.error('IMU sensor permission denied');
        // Show a user-friendly alert
        if (typeof alert === 'function') {
          setTimeout(() => {
            alert('IMU sensor access was denied. Please enable motion access in your device settings to use this feature.');
          }, 500);
        }
        return false;
      }
      
      console.log('Permission granted, setting up sensors...');
      
      // Setup event handlers for accelerometer and gyroscope
      this.setupAccelerometerAndGyroscope();
      
      // Start sending data at the specified sample rate
      this.intervalId = setInterval(() => {
        this.sendSensorData();
      }, 1000 / this.sampleRate);
      
      const deviceType = this.isIOS ? 'iOS' : 'Android';
      console.log(`${deviceType} IMU sensor manager initialized successfully`);
      return true;
    } catch (error) {
      console.error('Error initializing IMU sensor:', error);
      return false;
    }
  }

  // Start IMU data collection and transmission without requesting permission (assumes already granted)
  async startIMUSensorWithoutPermission(websocket, isSessionActive) {
    try {
      this.ws = websocket;  // Use consistent property name
      this.websocket = websocket;  // Keep backup for compatibility
      this.isActive = isSessionActive;
      
      // Check if we already have permission (for iOS) or if we're on Android
      if (this.isIOS && !this.permissionGranted) {
        console.warn('Permission not granted for iOS IMU sensor, cannot start without permission');
        return false;
      }
      
      // For Android or when iOS permission is already granted, proceed directly
      console.log('Starting IMU sensor with existing permissions...');
      
      // Setup event handlers for accelerometer and gyroscope
      this.setupAccelerometerAndGyroscope();
      
      // Start sending data at the specified sample rate
      this.intervalId = setInterval(() => {
        this.sendSensorData();
      }, 1000 / this.sampleRate);
      
      const deviceType = this.isIOS ? 'iOS' : 'Android';
      console.log(`${deviceType} IMU sensor started successfully with existing permissions`);
      return true;
    } catch (error) {
      console.error('Error starting IMU sensor without permission request:', error);
      return false;
    }
  }

  // Set up accelerometer and gyroscope listeners
  setupAccelerometerAndGyroscope() {
    window.addEventListener('devicemotion', (event) => {
      if (!this.isActive) return;
      
      // Get accelerometer data (in m/s²)
      if (event.acceleration) {
        // Both iOS and Android provide the same accelerometer data format
        this.accelerometerData = {
          x: event.acceleration.x || 0,
          y: event.acceleration.y || 0,
          z: event.acceleration.z || 0
        };
      } else if (event.accelerationIncludingGravity && this.isAndroid) {
        // Some Android devices may only provide accelerationIncludingGravity
        // This is a fallback, but note that this includes gravity which might need filtering
        this.accelerometerData = {
          x: event.accelerationIncludingGravity.x || 0,
          y: event.accelerationIncludingGravity.y || 0,
          z: event.accelerationIncludingGravity.z || 0
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

  // Send collected sensor data through WebSocket
  sendSensorData() {
    // Support both ws and websocket properties for backward compatibility
    const socket = this.ws || this.websocket;
    if (!this.isActive || !socket || socket.readyState !== WebSocket.OPEN) {
      return;
    }

    const timestamp = Date.now();
    
    // Create a structured payload with only accelerometer and gyroscope data
    const payload = {
      imu: {
        timestamp: timestamp,
        accelerometer: this.accelerometerData,
        gyroscope: this.gyroscopeData
      }
    };

    // Send as JSON
    try {
      socket.send(JSON.stringify(payload));
    } catch (error) {
      console.error('Error sending IMU data:', error);
    }
  }

  // Check Android sensor availability
  checkAndroidSensorAvailability() {
    // Check if the device supports the required sensor events
    const hasDeviceMotion = 'ondevicemotion' in window;
    
    if (!hasDeviceMotion) {
      console.warn('Android device missing required DeviceMotion sensor events');
      return false;
    }
    
    return true;
  }

  // Stop IMU data collection
  stopIMUSensor() {
    this.isActive = false;
    
    if (this.intervalId) {
      clearInterval(this.intervalId);
      this.intervalId = null;
    }
    
    const deviceType = this.isIOS ? 'iOS' : this.isAndroid ? 'Android' : 'Unknown';
    console.log(`${deviceType} IMU sensor stopped`);
    return Promise.resolve();
  }
}

// Make the IMU manager available globally
window.IMUSensorManager = IMUSensorManager;
