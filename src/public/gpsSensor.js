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
    
    // GPS options - optimized for different platforms
    this.gpsOptions = {
      enableHighAccuracy: true,
      timeout: this.isIOS ? 15000 : 10000, // Longer timeout for iOS
      maximumAge: this.isIOS ? 2000 : 500 // Accept slightly older data on iOS
    };
    
    // Log initialization for debugging
    console.log('GPS Sensor Manager initialized:', {
      isIOS: this.isIOS,
      isAndroid: this.isAndroid,
      isHTTPS: location.protocol === 'https:',
      gpsOptions: this.gpsOptions
    });
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

    // iOS-specific checks
    if (this.isIOS) {
      console.log('iOS device detected - checking location permissions');
      
      // Check if we're running on HTTPS (required for iOS geolocation)
      if (location.protocol !== 'https:' && location.hostname !== 'localhost') {
        console.error('iOS requires HTTPS for geolocation access');
        if (typeof alert === 'function') {
          setTimeout(() => {
            alert('Location access requires HTTPS on iOS devices. Please use a secure connection.');
          }, 500);
        }
        return false;
      }
      
      // iOS Safari doesn't fully support the Permissions API for geolocation
      // Skip Permissions API check and go directly to getCurrentPosition
      console.log('iOS: Requesting location permission directly...');
    } else {
      // For non-iOS devices, try to use Permissions API if available
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
    }

    // Try to get current position to trigger permission request
    return new Promise((resolve) => {
      // Use a longer timeout for iOS as it may take longer to show permission dialog
      const iosOptions = this.isIOS ? {
        ...this.gpsOptions,
        timeout: 15000, // Longer timeout for iOS
        maximumAge: 60000 // Accept older cached position on iOS
      } : this.gpsOptions;
      
      console.log(`Requesting GPS permission with options:`, iosOptions);
      
      navigator.geolocation.getCurrentPosition(
        (position) => {
          console.log('GPS permission granted successfully');
          console.log('Initial position:', {
            lat: position.coords.latitude,
            lng: position.coords.longitude,
            accuracy: position.coords.accuracy
          });
          this.permissionGranted = true;
          resolve(true);
        },
        (error) => {
          console.error('GPS permission denied or error:', error.message);
          console.error('Error code:', error.code);
          console.error('Error details:', {
            PERMISSION_DENIED: error.PERMISSION_DENIED,
            POSITION_UNAVAILABLE: error.POSITION_UNAVAILABLE,
            TIMEOUT: error.TIMEOUT
          });
          
          // Provide iOS-specific error messages
          if (this.isIOS) {
            let iosMessage = 'Location access failed on iOS. ';
            switch (error.code) {
              case error.PERMISSION_DENIED:
                iosMessage += 'Please enable location access in Safari settings or device location settings.';
                break;
              case error.POSITION_UNAVAILABLE:
                iosMessage += 'Location services may be disabled. Please check your device settings.';
                break;
              case error.TIMEOUT:
                iosMessage += 'Location request timed out. Please try again in a location with better GPS signal.';
                break;
            }
            console.error(iosMessage);
          }
          
          this.permissionGranted = false;
          resolve(false);
        },
        iosOptions
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
    
    // Log device detection
    console.log('GPS Sensor startup info:', {
      isIOS: this.isIOS,
      isAndroid: this.isAndroid,
      userAgent: navigator.userAgent,
      protocol: location.protocol,
      hostname: location.hostname
    });
    
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
        // Show a user-friendly alert with iOS-specific instructions
        if (typeof alert === 'function') {
          const message = this.isIOS 
            ? 'GPS location access was denied. For iOS:\n1. Enable Location Services in Settings\n2. Allow location for Safari\n3. Refresh this page'
            : 'GPS location access was denied. Please enable location access in your device settings to use this feature.';
          
          setTimeout(() => {
            alert(message);
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
   * Start GPS data collection and transmission without requesting permission (assumes already granted)
   * @param {WebSocket} websocket - WebSocket connection to send data
   * @param {boolean} isSessionActive - Whether the sensor session is active
   * @returns {Promise<boolean>} - Success status
   */
  async startGPSSensorWithoutPermission(websocket, isSessionActive) {
    try {
      this.ws = websocket;
      this.isActive = isSessionActive;
      
      // Check if we already have permission (for iOS) or if we're on Android
      if (this.isIOS && !this.permissionGranted) {
        console.warn('Permission not granted for iOS GPS sensor, cannot start without permission');
        return false;
      }
      
      // For Android or when iOS permission is already granted, proceed directly
      console.log('Starting GPS sensor with existing permissions...');
      
      // Log device detection
      console.log('GPS Sensor startup info:', {
        isIOS: this.isIOS,
        isAndroid: this.isAndroid,
        permissionGranted: this.permissionGranted,
        protocol: location.protocol,
        hostname: location.hostname
      });

      if (!navigator.geolocation) {
        console.error('Geolocation is not supported by this browser');
        return false;
      }

      // Start watching position directly
      this.setupGPSTracking();
      
      const deviceType = this.isIOS ? 'iOS' : this.isAndroid ? 'Android' : 'Desktop';
      console.log(`${deviceType} GPS sensor started successfully with existing permissions`);
      return true;
    } catch (error) {
      console.error('Error starting GPS sensor without permission request:', error);
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
    
    // Use iOS-optimized settings if on iOS
    const trackingOptions = this.isIOS ? {
      enableHighAccuracy: true,
      timeout: 20000, // Longer timeout for iOS
      maximumAge: 2000 // Accept slightly older position data on iOS
    } : this.gpsOptions;
    
    console.log('GPS tracking options:', trackingOptions);
    
    this.watchId = navigator.geolocation.watchPosition(
      (position) => {
        this.handlePositionUpdate(position);
      },
      (error) => {
        this.handlePositionError(error);
      },
      trackingOptions
    );
    
    console.log(`GPS watch started with ID: ${this.watchId}`);
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

    // Log position updates for debugging (especially helpful for iOS)
    if (this.isIOS) {
      console.log('iOS GPS update:', {
        lat: coords.latitude?.toFixed(6),
        lng: coords.longitude?.toFixed(6),
        accuracy: coords.accuracy?.toFixed(2) + 'm',
        altitude: coords.altitude?.toFixed(2) + 'm' || 'null',
        timestamp: new Date(timestamp).toISOString()
      });
    }

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
    
    // Provide additional iOS-specific guidance
    if (this.isIOS) {
      console.error('iOS GPS troubleshooting:');
      console.error('1. Check if location services are enabled in device Settings > Privacy & Security > Location Services');
      console.error('2. Check if Safari has location permission in Settings > Safari > Location');
      console.error('3. Make sure you\'re using HTTPS (not HTTP)');
      console.error('4. Try refreshing the page and allowing location when prompted');
      
      if (error.code === error.PERMISSION_DENIED) {
        if (typeof alert === 'function') {
          setTimeout(() => {
            alert('Location permission denied. Please:\n1. Enable Location Services in iOS Settings\n2. Allow location access for Safari\n3. Refresh the page and try again');
          }, 1000);
        }
      }
    }
    
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

  /**
   * Test GPS capabilities and permissions (useful for debugging iOS issues)
   * @returns {Promise<Object>} - Test results
   */
  async testGPSCapabilities() {
    const results = {
      geolocationSupported: !!navigator.geolocation,
      isHTTPS: location.protocol === 'https:',
      permissionsAPI: 'permissions' in navigator,
      isIOS: this.isIOS,
      userAgent: navigator.userAgent,
      timestamp: new Date().toISOString()
    };
    
    console.log('GPS Capabilities Test Results:', results);
    
    if (navigator.geolocation) {
      try {
        console.log('Testing basic geolocation access...');
        const position = await new Promise((resolve, reject) => {
          navigator.geolocation.getCurrentPosition(resolve, reject, {
            enableHighAccuracy: false,
            timeout: 10000,
            maximumAge: 300000
          });
        });
        
        results.basicLocationTest = {
          success: true,
          latitude: position.coords.latitude,
          longitude: position.coords.longitude,
          accuracy: position.coords.accuracy
        };
        console.log('Basic location test successful:', results.basicLocationTest);
      } catch (error) {
        results.basicLocationTest = {
          success: false,
          error: error.message,
          code: error.code
        };
        console.error('Basic location test failed:', results.basicLocationTest);
      }
    }
    
    return results;
  }
}

// Make the GPS manager available globally
window.GPSSensorManager = GPSSensorManager;
