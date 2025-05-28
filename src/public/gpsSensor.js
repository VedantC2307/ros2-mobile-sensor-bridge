/**
 * GPS Sensor Manager
 * Handles GPS data from the device's location services
 * Supports both iOS and Android devices
 */

class GPSSensorManager {
  constructor() {
    this.isActive = false;
    this.ws = null;
    this.sampleRate = 2; // Hz - Try 2Hz, fallback to 1Hz if battery drain is high
    this.watchId = null;
    
    // Store GPS data
    this.gpsData = {
      latitude: 0,
      longitude: 0,
      altitude: 0,
      accuracy: 0,
      heading: 0,
      speed: 0,
      timestamp: 0
    };
    
    // Device detection
    this.isIOS = /iPad|iPhone|iPod/.test(navigator.userAgent) && !window.MSStream;
    this.isAndroid = /Android/.test(navigator.userAgent);
    
    // Permission status
    this.permissionGranted = false;
    
    // GPS options
    this.gpsOptions = {
      enableHighAccuracy: true,
      timeout: 10000, // 10 seconds timeout
      maximumAge: 500 // Accept cached position up to 0.5 seconds old
    };
  }

  /**
   * Request permission to access device location
   * Required for modern browsers due to privacy restrictions
   * @returns {Promise<boolean>} - Resolves when permission is granted, rejects when denied
   */
  async requestPermission() {
    if (!navigator.geolocation) {
      console.error('Geolocation is not supported by this browser');
      return false;
    }

    // For modern browsers, check permissions API if available
    if ('permissions' in navigator) {
      try {
        const permission = await navigator.permissions.query({ name: 'geolocation' });
        
        if (permission.state === 'granted') {
          this.permissionGranted = true;
          return true;
        } else if (permission.state === 'denied') {
          console.error('Geolocation permission denied');
          return false;
        }
        // If state is 'prompt', we'll request permission below
      } catch (error) {
        console.warn('Permissions API not fully supported, trying direct geolocation access');
      }
    }

    // Try to get current position to trigger permission request
    return new Promise((resolve) => {
      navigator.geolocation.getCurrentPosition(
        (position) => {
          console.log('GPS permission granted');
          this.permissionGranted = true;
          resolve(true);
        },
        (error) => {
          console.error('GPS permission denied or error:', error.message);
          this.permissionGranted = false;
          resolve(false);
        },
        this.gpsOptions
      );
    });
  }

  /**
   * Start GPS data collection and transmission
   * @param {WebSocket} websocket - WebSocket connection to send data
   * @param {boolean} isSessionActive - Whether the sensor session is active
   * @returns {Promise<boolean>} - Success status
   */
  async startGPSSensor(websocket, isSessionActive) {
    this.ws = websocket;
    this.isActive = isSessionActive;
    
    if (!this.isIOS && !this.isAndroid) {
      console.log('GPS sensor is available on all devices, but optimized for mobile');
    }

    if (!navigator.geolocation) {
      console.error('Geolocation is not supported by this browser');
      return false;
    }

    try {
      // Request permission if needed
      console.log('Starting GPS sensor and requesting permissions...');
      const permissionGranted = await this.requestPermission();
      
      if (!permissionGranted) {
        console.error('GPS sensor permission denied');
        // Show a user-friendly alert
        if (typeof alert === 'function') {
          setTimeout(() => {
            alert('GPS location access was denied. Please enable location access in your device settings to use this feature.');
          }, 500);
        }
        return false;
      }

      console.log('Permission granted, setting up GPS tracking...');
      
      // Start watching position
      this.setupGPSTracking();
      
      const deviceType = this.isIOS ? 'iOS' : this.isAndroid ? 'Android' : 'Desktop';
      console.log(`${deviceType} GPS sensor manager initialized successfully`);
      return true;
    } catch (error) {
      console.error('Error initializing GPS sensor:', error);
      return false;
    }
  }

  /**
   * Set up GPS position tracking using watchPosition
   */
  setupGPSTracking() {
    if (!navigator.geolocation || !this.isActive) {
      return;
    }

    // Clear any existing watch
    if (this.watchId !== null) {
      navigator.geolocation.clearWatch(this.watchId);
    }

    console.log('Starting GPS position tracking...');
    
    this.watchId = navigator.geolocation.watchPosition(
      (position) => {
        this.handlePositionUpdate(position);
      },
      (error) => {
        this.handlePositionError(error);
      },
      this.gpsOptions
    );
  }

  /**
   * Handle successful position update
   * @param {GeolocationPosition} position - Position object from geolocation API
   */
  handlePositionUpdate(position) {
    if (!this.isActive || !this.ws || this.ws.readyState !== WebSocket.OPEN) {
      return;
    }

    const coords = position.coords;
    const timestamp = position.timestamp || Date.now();

    // Update stored GPS data
    this.gpsData = {
      latitude: coords.latitude || 0,
      longitude: coords.longitude || 0,
      altitude: coords.altitude || 0, // Can be null, we'll handle this in ROS
      accuracy: coords.accuracy || 0,
      heading: coords.heading || 0, // Can be null, we'll store but not use in NavSatFix
      speed: coords.speed || 0, // Can be null, we'll store but not use in NavSatFix
      timestamp: timestamp
    };

    // Send GPS data immediately when received
    this.sendGPSData();
  }

  /**
   * Handle position error
   * @param {GeolocationPositionError} error - Error object from geolocation API
   */
  handlePositionError(error) {
    let errorMessage = 'GPS position error: ';
    
    switch (error.code) {
      case error.PERMISSION_DENIED:
        errorMessage += 'Permission denied';
        this.permissionGranted = false;
        break;
      case error.POSITION_UNAVAILABLE:
        errorMessage += 'Position unavailable';
        break;
      case error.TIMEOUT:
        errorMessage += 'Request timeout';
        break;
      default:
        errorMessage += 'Unknown error';
        break;
    }
    
    console.error(errorMessage, error.message);
    
    // If permission is denied, we should stop the sensor
    if (error.code === error.PERMISSION_DENIED) {
      this.stopGPSSensor();
    }
  }

  /**
   * Send collected GPS data through WebSocket
   */
  sendGPSData() {
    if (!this.isActive || !this.ws || this.ws.readyState !== WebSocket.OPEN) {
      return;
    }

    // Create a structured payload with GPS data
    const payload = {
      gps: {
        latitude: this.gpsData.latitude,
        longitude: this.gpsData.longitude,
        altitude: this.gpsData.altitude,
        accuracy: this.gpsData.accuracy,
        heading: this.gpsData.heading,
        speed: this.gpsData.speed,
        timestamp: this.gpsData.timestamp
      }
    };

    // Send as JSON
    try {
      this.ws.send(JSON.stringify(payload));
    } catch (error) {
      console.error('Error sending GPS data:', error);
    }
  }

  /**
   * Stop GPS data collection
   */
  stopGPSSensor() {
    console.log('Stopping GPS sensor...');
    
    this.isActive = false;
    
    // Clear GPS watch if active
    if (this.watchId !== null) {
      navigator.geolocation.clearWatch(this.watchId);
      this.watchId = null;
      console.log('GPS position tracking stopped');
    }
    
    // Reset GPS data
    this.gpsData = {
      latitude: 0,
      longitude: 0,
      altitude: 0,
      accuracy: 0,
      heading: 0,
      speed: 0,
      timestamp: 0
    };
    
    console.log('GPS sensor stopped successfully');
  }
}

// Make the GPS manager available globally
window.GPSSensorManager = GPSSensorManager;
