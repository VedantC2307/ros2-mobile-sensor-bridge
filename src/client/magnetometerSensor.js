/**
 * Magnetometer Sensor Manager
 * Handles magnetic field data from the device's magnetometer
 *
 * Data sources (in priority order):
 *   1. Generic Sensor API (Magnetometer) - Chrome/Android, gives raw x/y/z in µT
 *   2. DeviceOrientationEvent (absolute) - fallback, gives compass heading
 *      On iOS: webkitCompassHeading is used directly
 *
 * Published ROS message: sensor_msgs/msg/MagneticField
 */

class MagnetometerSensorManager {
  constructor() {
    this.isActive = false;
    this.ws = null;
    this.sampleRate = 10; // Hz - default, updated from config
    this.intervalId = null;

    // Latest reading (in Tesla for Magnetometer API, unit-vector for orientation fallback)
    this.magnetometerData = { x: 0, y: 0, z: 0 };
    this.dataSource = null; // 'api' | 'orientation'

    // Magnetometer API sensor instance
    this.sensor = null;

    // DeviceOrientation handler reference (needed for removeEventListener)
    this._orientationHandler = null;

    // Device detection
    this.isIOS = /iPad|iPhone|iPod/.test(navigator.userAgent) && !window.MSStream;
    this.isAndroid = /Android/.test(navigator.userAgent);

    this.permissionGranted = false;

    this.loadConfig();
  }

  // -------------------------------------------------------------------------
  // Configuration
  // -------------------------------------------------------------------------

  async loadConfig() {
    try {
      const response = await fetch('/api/config');
      if (response.ok) {
        const config = await response.json();
        if (config.magnetometer && config.magnetometer.sample_rate) {
          this.sampleRate = config.magnetometer.sample_rate;
          console.log('Magnetometer sample rate loaded from config:', this.sampleRate, 'Hz');
        }
      }
    } catch (error) {
      console.warn('Failed to load magnetometer config, using default sample rate:', this.sampleRate);
    }
  }

  // -------------------------------------------------------------------------
  // Availability check
  // -------------------------------------------------------------------------

  /**
   * Returns which data source is available on this device.
   * @returns {'api'|'orientation'|null}
   */
  checkAvailability() {
    if (typeof Magnetometer !== 'undefined') {
      return 'api';
    }
    if ('ondeviceorientationabsolute' in window || 'ondeviceorientation' in window) {
      return 'orientation';
    }
    return null;
  }

  // -------------------------------------------------------------------------
  // Permission handling
  // -------------------------------------------------------------------------

  async requestPermission() {
    const source = this.checkAvailability();

    if (source === 'api') {
      // Generic Sensor API - check via Permissions API
      if (navigator.permissions) {
        try {
          const result = await navigator.permissions.query({ name: 'magnetometer' });
          if (result.state === 'denied') {
            console.warn('Magnetometer permission denied via Permissions API');
            return false;
          }
        } catch (e) {
          // Permissions API may not support 'magnetometer' on all browsers - proceed anyway
          console.log('Permissions API query for magnetometer not supported, proceeding');
        }
      }
      this.permissionGranted = true;
      return true;
    }

    if (source === 'orientation') {
      // iOS 13+ requires explicit permission for DeviceOrientationEvent
      if (this.isIOS && typeof DeviceOrientationEvent !== 'undefined' &&
          typeof DeviceOrientationEvent.requestPermission === 'function') {
        try {
          console.log('Requesting DeviceOrientationEvent permission for magnetometer (iOS)...');
          const state = await DeviceOrientationEvent.requestPermission();
          if (state === 'granted') {
            console.log('DeviceOrientationEvent permission granted');
            this.permissionGranted = true;
            return true;
          } else {
            console.warn('DeviceOrientationEvent permission denied:', state);
            return false;
          }
        } catch (error) {
          console.error('Error requesting DeviceOrientationEvent permission:', error);
          return false;
        }
      }
      // Non-iOS or no permission API needed
      this.permissionGranted = true;
      return true;
    }

    console.warn('No magnetometer source available on this device');
    return false;
  }

  // -------------------------------------------------------------------------
  // Sensor setup
  // -------------------------------------------------------------------------

  _setupMagnetometerAPI() {
    try {
      this.sensor = new Magnetometer({ frequency: this.sampleRate });
      this.sensor.addEventListener('reading', () => {
        if (!this.isActive) return;
        // Convert from microtesla (µT) to Tesla
        this.magnetometerData = {
          x: (this.sensor.x || 0) * 1e-6,
          y: (this.sensor.y || 0) * 1e-6,
          z: (this.sensor.z || 0) * 1e-6
        };
      });
      this.sensor.addEventListener('error', (event) => {
        console.error('Magnetometer sensor error:', event.error);
      });
      this.sensor.start();
      this.dataSource = 'api';
      console.log('Magnetometer: using Generic Sensor API');
    } catch (error) {
      console.error('Failed to start Magnetometer API:', error);
      throw error;
    }
  }

  _setupOrientationFallback() {
    this._orientationHandler = (event) => {
      if (!this.isActive) return;

      let heading = null;

      if (event.webkitCompassHeading !== undefined && event.webkitCompassHeading !== null) {
        // iOS: degrees clockwise from North (0–360)
        heading = event.webkitCompassHeading;
      } else if (event.absolute && event.alpha !== null) {
        // Android absolute orientation: alpha is CCW from North
        heading = (360 - event.alpha) % 360;
      } else if (event.alpha !== null) {
        // Non-absolute DeviceOrientationEvent (compass heading approximation)
        heading = (360 - event.alpha) % 360;
      }

      if (heading !== null) {
        const headingRad = (heading * Math.PI) / 180;
        // Produce a unit direction vector pointing toward magnetic North
        // NOTE: z=0 (no tilt compensation); covariance on the server side
        //       will be set to -1 to indicate unknown magnitude.
        this.magnetometerData = {
          x: Math.sin(headingRad),
          y: Math.cos(headingRad),
          z: 0
        };
      }
    };

    if ('ondeviceorientationabsolute' in window) {
      window.addEventListener('deviceorientationabsolute', this._orientationHandler);
      console.log('Magnetometer: using deviceorientationabsolute event');
    } else {
      window.addEventListener('deviceorientation', this._orientationHandler);
      console.log('Magnetometer: using deviceorientation event (non-absolute)');
    }

    this.dataSource = 'orientation';
  }

  // -------------------------------------------------------------------------
  // Start / stop
  // -------------------------------------------------------------------------

  async startMagnetometerSensor(websocket, isSessionActive) {
    this.ws = websocket;
    this.isActive = isSessionActive;

    const source = this.checkAvailability();
    if (!source) {
      console.error('No magnetometer source available on this device');
      return false;
    }

    const permissionGranted = await this.requestPermission();
    if (!permissionGranted) {
      console.error('Magnetometer permission denied');
      if (typeof alert === 'function') {
        setTimeout(() => {
          alert('Magnetometer access was denied. Please enable the required sensor permissions in your device settings.');
        }, 500);
      }
      return false;
    }

    try {
      if (source === 'api') {
        this._setupMagnetometerAPI();
      } else {
        this._setupOrientationFallback();
      }
    } catch (error) {
      console.error('Error setting up magnetometer sensor:', error);
      return false;
    }

    // For the orientation fallback the event fires continuously; for the
    // Sensor API the 'reading' event handles updates.  Either way we send
    // at the configured sample rate.
    this.intervalId = setInterval(() => {
      this._sendData();
    }, 1000 / this.sampleRate);

    console.log(`Magnetometer sensor started (source: ${this.dataSource}, ${this.sampleRate} Hz)`);
    return true;
  }

  async startMagnetometerSensorWithoutPermission(websocket, isSessionActive) {
    this.ws = websocket;
    this.isActive = isSessionActive;

    if (this.isIOS && !this.permissionGranted) {
      console.warn('Permission not yet granted for magnetometer (iOS), cannot start without permission');
      return false;
    }

    const source = this.checkAvailability();
    if (!source) {
      console.error('No magnetometer source available on this device');
      return false;
    }

    try {
      if (source === 'api') {
        this._setupMagnetometerAPI();
      } else {
        this._setupOrientationFallback();
      }
    } catch (error) {
      console.error('Error setting up magnetometer sensor:', error);
      return false;
    }

    this.intervalId = setInterval(() => {
      this._sendData();
    }, 1000 / this.sampleRate);

    console.log(`Magnetometer sensor started with existing permissions (source: ${this.dataSource})`);
    return true;
  }

  stopMagnetometerSensor() {
    this.isActive = false;

    if (this.intervalId) {
      clearInterval(this.intervalId);
      this.intervalId = null;
    }

    // Stop Magnetometer API sensor
    if (this.sensor) {
      try {
        this.sensor.stop();
      } catch (e) {
        // ignore
      }
      this.sensor = null;
    }

    // Remove orientation event listener
    if (this._orientationHandler) {
      window.removeEventListener('deviceorientationabsolute', this._orientationHandler);
      window.removeEventListener('deviceorientation', this._orientationHandler);
      this._orientationHandler = null;
    }

    console.log('Magnetometer sensor stopped');
    return Promise.resolve();
  }

  // -------------------------------------------------------------------------
  // Data transmission
  // -------------------------------------------------------------------------

  _sendData() {
    if (!this.isActive || !this.ws || this.ws.readyState !== WebSocket.OPEN) return;

    const payload = {
      magnetometer: {
        timestamp: Date.now(),
        x: this.magnetometerData.x,
        y: this.magnetometerData.y,
        z: this.magnetometerData.z,
        source: this.dataSource // 'api' | 'orientation'
      }
    };

    try {
      this.ws.send(JSON.stringify(payload));
    } catch (error) {
      console.error('Error sending magnetometer data:', error);
    }
  }
}

window.MagnetometerSensorManager = MagnetometerSensorManager;
