class IOSCameraManager {
    constructor() {
        this.cameraStarted = false;
        this.videoTrack = null;
        this.lastCameraFrame = null;
        this.cameraConfig = {
            facingMode: "environment", // Default value until config is loaded
            quality: 0.7,  // Default quality until config is loaded
            fps: 20       // Default fps until config is loaded
        };
        this.availableCameras = [];
        this.selectedCameraId = null;
        this.devicePermissionGranted = false;
        this.videoElement = null;
        this._captureInterval = null;
        
        // Fixed dimensions that are consistent across all capture methods
        this.fixedWidth = 480;
        this.fixedHeight = 640;
        
        // Fetch camera configuration when created
        this.fetchCameraConfig();
        
        console.log('iOS Camera Manager initialized');
    }
    
    // New method to scan for available cameras but not create UI elements
    async scanAvailableCameras() {
        try {
            if (!navigator.mediaDevices || !navigator.mediaDevices.enumerateDevices) {
                throw new Error('enumerateDevices API not supported');
            }
            
            // On iOS devices, we need explicit permission before labels are available
            if (!this.devicePermissionGranted) {
                try {
                    // Request camera access with the preferred facing mode from config
                    console.log(`Requesting camera permission with facing mode: ${this.cameraConfig.facingMode}`);
                    const tempStream = await navigator.mediaDevices.getUserMedia({ 
                        video: { facingMode: this.cameraConfig.facingMode } 
                    });
                    this.devicePermissionGranted = true;
                    
                    // Stop the temporary stream immediately
                    tempStream.getTracks().forEach(track => track.stop());
                    console.log('Camera permission granted, can now enumerate devices with labels');
                } catch (permErr) {
                    console.warn('Could not get camera permission for enumeration:', permErr);
                    // Continue anyway, but labels might not be available
                }
            }
            
            // Now get the devices - after permission, labels should be available
            const devices = await navigator.mediaDevices.enumerateDevices();
            this.availableCameras = devices.filter(device => device.kind === 'videoinput');
            console.log('Available cameras (iOS):', this.availableCameras);
            
            // Generate labels and identify camera types for iOS devices
            if (this.availableCameras.length > 1) {
                this.availableCameras.forEach((camera, index) => {
                    if (!camera.label) {
                        // iOS typically has front camera at index 0, back at index 1
                        if (index === 0) {
                            camera._generatedLabel = 'Front Camera';
                            camera._cameraType = 'user';
                        } else if (index === 1) {
                            camera._generatedLabel = 'Back Camera';
                            camera._cameraType = 'environment';
                        } else {
                            camera._generatedLabel = `Camera ${index + 1}`;
                            camera._cameraType = 'unknown';
                        }
                    } else {
                        // If we have labels, try to determine camera type
                        if (camera.label.toLowerCase().includes('front')) {
                            camera._cameraType = 'user';
                        } else if (camera.label.toLowerCase().includes('back') || 
                                camera.label.toLowerCase().includes('rear')) {
                            camera._cameraType = 'environment';
                        }
                    }
                });
                
                // Try to find the camera that matches our config
                const preferredCamera = this.availableCameras.find(
                    camera => camera._cameraType === this.cameraConfig.facingMode
                );
                
                // If we found a matching camera, pre-select it
                if (preferredCamera) {
                    this.selectedCameraId = preferredCamera.deviceId;
                    console.log(`Pre-selected ${preferredCamera._generatedLabel || preferredCamera.label} based on config`);
                }
            }
            
            // Populate the dropdown in the HTML
            this.populateCameraDropdown();
            
            // Select appropriate camera by default based on config
            if (this.availableCameras.length > 0 && !this.selectedCameraId) {
                if (this.cameraConfig.facingMode === "environment" && this.availableCameras.length > 1) {
                    // For iOS, typically back camera (environment) is at index 1
                    this.selectedCameraId = this.availableCameras[1].deviceId;
                    console.log('Using back camera (environment mode) from config');
                } else {
                    // If user-facing or only one camera available, use the first one
                    this.selectedCameraId = this.availableCameras[0].deviceId;
                }
            }
            
            return this.availableCameras;
        } catch (error) {
            console.error('Failed to scan cameras (iOS):', error);
            return [];
        }
    }
    
    // Method to populate the dropdown that already exists in HTML
    populateCameraDropdown() {
        const dropdown = document.getElementById('camera-dropdown');
        if (!dropdown) return;
        
        // Clear existing options
        dropdown.innerHTML = '';
        
        if (this.availableCameras.length === 0) {
            const option = document.createElement('option');
            option.text = 'No cameras found';
            dropdown.add(option);
            return;
        }
        
        // Add options for each camera
        this.availableCameras.forEach((camera, index) => {
            const option = document.createElement('option');
            option.value = camera.deviceId;
            
            // Use label if available or generated label
            if (camera._generatedLabel) {
                option.text = camera._generatedLabel;
            } else if (camera.label) {
                if (camera.label.toLowerCase().includes('front')) {
                    option.text = `Front Camera (${index + 1})`;
                } else if (camera.label.toLowerCase().includes('back') || 
                           camera.label.toLowerCase().includes('rear')) {
                    option.text = `Back Camera (${index + 1})`;
                } else {
                    option.text = camera.label;
                }
            } else {
                option.text = `Camera ${index + 1}`;
            }
            
            dropdown.add(option);
        });
        
        // Set the currently selected camera
        if (this.selectedCameraId) {
            dropdown.value = this.selectedCameraId;
        }
        
        // Add event listener to handle camera change
        dropdown.addEventListener('change', (e) => {
            this.selectedCameraId = e.target.value;
            console.log(`Selected camera: ${e.target.options[e.target.selectedIndex].text} (${this.selectedCameraId})`);
            
            // Restart camera with new selection if already started
            if (this.cameraStarted) {
                this.stopCamera();
                // Get WebSocket and session status from parent context
                const cameraWs = window.cameraWs; 
                const isSessionActive = document.body.classList.contains('session-active');
                if (cameraWs && isSessionActive) {
                    this.startCamera(cameraWs, isSessionActive);
                }
            }
        });
    }
    
    // Method to switch camera externally
    switchCamera(deviceId) {
        if (deviceId && deviceId !== this.selectedCameraId) {
            this.selectedCameraId = deviceId;
            console.log(`Switching to camera with ID: ${deviceId}`);
            
            if (this.cameraStarted) {
                this.stopCamera();
                const cameraWs = window.cameraWs;
                const isSessionActive = document.body.classList.contains('session-active');
                if (cameraWs && isSessionActive) {
                    this.startCamera(cameraWs, isSessionActive);
                }
            }
        }
    }
    
    // Method to fetch camera configuration from the server
    async fetchCameraConfig() {
        try {
            const response = await fetch('/api/config');
            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }
            const config = await response.json();
            
            // Set facing mode if available
            if (config.camera && config.camera.facingMode) {
                this.cameraConfig.facingMode = config.camera.facingMode;
                console.log('Using camera facing mode from config:', this.cameraConfig.facingMode);
            }
            
            if (config.camera) {
                if (config.camera.quality !== undefined) {
                    this.cameraConfig.quality = parseFloat(config.camera.quality);
                    console.log('Using camera quality from config:', this.cameraConfig.quality);
                } else if (config.camera.quaity !== undefined) {
                    // Fallback for the current typo in the config file
                    this.cameraConfig.quality = parseFloat(config.camera.quaity);
                    console.log('Using camera quality from config (with typo):', this.cameraConfig.quality);
                }
            }
            
            // Set fps if available, with maximum of 30
            if (config.camera && config.camera.fps !== undefined) {
                // Apply the maximum 30 fps limit
                this.cameraConfig.fps = Math.min(parseInt(config.camera.fps), 30);
                console.log('Using camera fps from config (limited to 30 max):', this.cameraConfig.fps);
            }
        } catch (error) {
            console.error('Failed to load camera config:', error);
            // Keep using the defaults
        }
    }

    async startCamera(ws, isSessionActive) {
        if (this.cameraStarted || !isSessionActive) return;
        try {
            if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
                throw new Error('getUserMedia API not supported');
            }
            
            // Scan for cameras if not already done or if we need to refresh the list
            if (this.availableCameras.length === 0 || !this.devicePermissionGranted) {
                await this.scanAvailableCameras();
            }

            // Check if 3D Position is enabled and override to user facing if it is
            const poseEnabled = document.getElementById('pose-select').checked;
            let videoConstraints = {};
            
            if (this.selectedCameraId) {
                // Use selected camera device ID
                console.log(`Starting camera with device ID: ${this.selectedCameraId}`);
                videoConstraints = {
                    deviceId: { exact: this.selectedCameraId },
                    width: { ideal: 1280 },
                    height: { ideal: 720 }
                };
            } else {
                // Fall back to facing mode if no camera selected
                const facingMode = poseEnabled ? "user" : this.cameraConfig.facingMode;
                console.log(`Starting camera with facing mode: ${facingMode} (pose enabled: ${poseEnabled})`);
                videoConstraints = { 
                    facingMode: facingMode,
                    width: { ideal: 1280 },
                    height: { ideal: 720 }
                };
            }

            const stream = await navigator.mediaDevices.getUserMedia({
                video: videoConstraints,
                audio: false
            });
            this.videoTrack = stream.getVideoTracks()[0];
            
            if (this.videoTrack) {
                // Log constraints for debugging
                console.log('Active camera settings:', this.videoTrack.getSettings());
                
                // Update camera selection interface after permission
                if (this.videoTrack.label) {
                    console.log(`Camera started: ${this.videoTrack.label}`);
                    
                    // Refresh camera list to update the interface with proper labels
                    await this.scanAvailableCameras();
                }
            }

            // Use fallback for Safari/iOS
            this.startCameraWithVideoCanvas(stream, ws);

            this.cameraStarted = true;
        } catch (err) {
            console.error('iOS Camera error:', err);
            alert('Camera access failed: ' + err.message);
        }
    }

    // Method for Safari/iOS using video+canvas approach
    startCameraWithVideoCanvas(stream, ws) {
        console.log('Using video+canvas method for iOS camera streaming');
        
        // Ensure only one capture interval is running
        if (this._captureInterval) {
            clearInterval(this._captureInterval);
            this._captureInterval = null;
            console.log('Cleared previous capture interval');
        }
        
        // Create hidden video element if it doesn't exist
        if (!this.videoElement) {
            this.videoElement = document.createElement('video');
            this.videoElement.style.display = 'none';
            this.videoElement.style.position = 'absolute';
            this.videoElement.style.left = '-9999px';
            this.videoElement.muted = true;
            this.videoElement.playsInline = true;
            this.videoElement.autoplay = true;
            document.body.appendChild(this.videoElement);
        }
        
        // Use consistent dimensions
        console.log(`Creating DOM canvas with dimensions: ${this.fixedWidth}x${this.fixedHeight}`);
        const canvas = document.createElement('canvas');
        canvas.width = this.fixedWidth;
        canvas.height = this.fixedHeight;
        const ctx = canvas.getContext('2d', { willReadFrequently: true });
        
        // Set video source to stream
        this.videoElement.srcObject = stream;
        
        let lastSentTime = 0;
        const desiredFps = Math.min(this.cameraConfig.fps, 30);
        const frameInterval = 1000 / desiredFps;
        
        // Function to start capturing frames
        const startCapturing = () => {
            console.log('Starting video frame capture for iOS/Safari');
            
            // Set up interval to capture frames
            const captureInterval = setInterval(() => {
                try {
                    const currentTime = performance.now();
                    if (currentTime - lastSentTime < frameInterval) return;
                    
                    if (ws && ws.readyState === WebSocket.OPEN && 
                        this.videoElement && 
                        this.videoElement.readyState === this.videoElement.HAVE_ENOUGH_DATA) {
                        
                        // Draw current video frame to canvas
                        const videoWidth = this.videoElement.videoWidth;
                        const videoHeight = this.videoElement.videoHeight;
                        
                        if (videoWidth && videoHeight) {
                            // Scale to fit canvas while maintaining aspect ratio
                            const scale = Math.min(canvas.width / videoWidth, canvas.height / videoHeight);
                            const x = (canvas.width - videoWidth * scale) / 2;
                            const y = (canvas.height - videoHeight * scale) / 2;
                            
                            ctx.clearRect(0, 0, canvas.width, canvas.height);
                            ctx.drawImage(this.videoElement, x, y, videoWidth * scale, videoHeight * scale);
                            
                            try {
                                // Convert canvas to base64 JPEG using quality from config
                                const dataUrl = canvas.toDataURL('image/jpeg', this.cameraConfig.quality);
                                this.lastCameraFrame = dataUrl;
                                
                                ws.send(JSON.stringify({
                                    timestamp: Date.now(),
                                    camera: dataUrl,
                                    width: this.fixedWidth,
                                    height: this.fixedHeight
                                }));
                                lastSentTime = currentTime;
                            } catch (canvasErr) {
                                console.error('Canvas to dataURL error:', canvasErr);
                            }
                        }
                    }
                } catch (err) {
                    console.error('Frame capture error:', err);
                }
            }, Math.floor(frameInterval / 2)); // Interval slightly faster than FPS to account for processing time
            
            // Store interval ID for cleanup
            this._captureInterval = captureInterval;
        };
        
        // Start video playback with proper error handling
        const playPromise = this.videoElement.play();
        if (playPromise !== undefined) {
            playPromise
                .then(() => {
                    console.log('Video playback started for iOS camera method');
                    
                    // For iOS Safari, we need to wait a bit to make sure video is actually playing
                    setTimeout(startCapturing, 500);
                })
                .catch(err => {
                    console.error('Video playback failed:', err);
                    
                    // Direct media autoplay is not allowed - iOS security restriction
                    console.log('iOS requires user interaction to play media');
                    
                    // Instead of creating a button, add a one-time click handler to the document
                    // that will try to start the video on the next user interaction
                    const startOnNextInteraction = () => {
                        this.videoElement.play()
                            .then(() => {
                                console.log('Video playback started after user interaction');
                                startCapturing();
                                document.removeEventListener('click', startOnNextInteraction);
                            })
                            .catch(playErr => {
                                console.error('Video play failed even after user interaction:', playErr);
                            });
                    };
                    
                    // Add the event listener
                    document.addEventListener('click', startOnNextInteraction, { once: true });
                    
                    // Show a notification to the user
                    const notification = document.createElement('div');
                    notification.style.position = 'fixed';
                    notification.style.bottom = '20px';
                    notification.style.left = '20px';
                    notification.style.right = '20px';
                    notification.style.backgroundColor = 'rgba(0, 0, 0, 0.7)';
                    notification.style.color = 'white';
                    notification.style.padding = '10px';
                    notification.style.borderRadius = '5px';
                    notification.style.textAlign = 'center';
                    notification.style.zIndex = '9999';
                    
                    document.body.appendChild(notification);
                    
                    // Remove the notification after 5 seconds or when clicked
                    setTimeout(() => {
                        if (notification.parentNode) {
                            notification.parentNode.removeChild(notification);
                        }
                    }, 5000);
                    
                    notification.addEventListener('click', () => {
                        if (notification.parentNode) {
                            notification.parentNode.removeChild(notification);
                        }
                    });
                });
        } else {
            console.log('Play promise not supported, starting capture immediately');
            startCapturing();
        }
    }

    stopCamera() {
        console.log('Stopping iOS camera and cleaning up resources');
        
        if (this.videoTrack) {
            this.videoTrack.stop();
            this.videoTrack = null;
            console.log('Video track stopped');
        }
        
        // Clean up video element and interval for Safari fallback
        if (this._captureInterval) {
            clearInterval(this._captureInterval);
            this._captureInterval = null;
            console.log('Capture interval cleared');
        }
        
        if (this.videoElement) {
            if (this.videoElement.srcObject) {
                const tracks = this.videoElement.srcObject.getTracks();
                tracks.forEach(track => track.stop());
                this.videoElement.srcObject = null;
                console.log('Video element tracks stopped');
            }
        }
        
        this.cameraStarted = false;
        this.lastCameraFrame = null;
        console.log('iOS camera fully stopped');
    }

    getLastFrame() {
        const frame = this.lastCameraFrame;
        this.lastCameraFrame = null;
        return frame;
    }
}

// Export the iOS camera manager
window.IOSCameraManager = IOSCameraManager;
