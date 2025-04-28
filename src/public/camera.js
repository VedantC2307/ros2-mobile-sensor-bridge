class CameraManager {
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
        this.isSafari = this.detectSafari();
        this._captureInterval = null;
        this._processorRunning = false;
        // Fixed dimensions that are consistent across all capture methods
        this.fixedWidth = 480;
        this.fixedHeight = 640;
        
        // Fetch camera configuration when created
        this.fetchCameraConfig();
    }
    
    // Detect Safari browser (including iOS Safari)
    detectSafari() {
        const ua = navigator.userAgent.toLowerCase();
        const isSafari = (ua.indexOf('safari') !== -1 && ua.indexOf('chrome') === -1) || 
                         /iphone|ipod|ipad/i.test(ua);
        console.log('Browser detected as ' + (isSafari ? 'Safari/iOS' : 'Chrome/Android'));
        return isSafari;
    }
    
    // New method to scan for available cameras but not create UI elements
    async scanAvailableCameras() {
        try {
            if (!navigator.mediaDevices || !navigator.mediaDevices.enumerateDevices) {
                throw new Error('enumerateDevices API not supported');
            }
            
            // On Pixel and some other devices, we need explicit permission before labels are available
            if (!this.devicePermissionGranted) {
                try {
                    // Request general camera access first to get permission
                    console.log('Requesting camera permission to enumerate devices...');
                    const tempStream = await navigator.mediaDevices.getUserMedia({ video: true });
                    this.devicePermissionGranted = true;
                    
                    // Stop the temporary stream immediately
                    tempStream.getTracks().forEach(track => track.stop());
                    console.log('Camera permission granted, can now enumerate devices with labels');
                } catch (permErr) {
                    console.warn('Could not get camera permission for enumeration:', permErr);
                    // Continue anyway, but labels might not be available
                }
            }
            
            // Now get the devices - after permission, labels should be available on most phones
            const devices = await navigator.mediaDevices.enumerateDevices();
            this.availableCameras = devices.filter(device => device.kind === 'videoinput');
            console.log('Available cameras:', this.availableCameras);
            
            // If we have multiple cameras but no labels (common on Pixel), add generic labels
            if (this.availableCameras.length > 1) {
                this.availableCameras.forEach((camera, index) => {
                    if (!camera.label) {
                        // Most Android phones have rear camera at index 0, front at index 1
                        if (index === 0) {
                            camera._generatedLabel = 'Rear Camera';
                        } else if (index === 1) {
                            camera._generatedLabel = 'Front Camera';
                        } else {
                            camera._generatedLabel = `Camera ${index + 1}`;
                        }
                    }
                });
            }
            
            // Instead of creating our own UI, populate the dropdown in the HTML
            this.populateCameraDropdown();
            
            // Select first camera by default if available
            if (this.availableCameras.length > 0 && !this.selectedCameraId) {
                this.selectedCameraId = this.availableCameras[0].deviceId;
            }
            
            return this.availableCameras;
        } catch (error) {
            console.error('Failed to scan cameras:', error);
            return [];
        }
    }
    
    // New method to populate the dropdown that already exists in HTML
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
    
    // New method to fetch camera configuration from the server
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
                    
                    // On Google Pixel, we might now have labels that we didn't have before
                    // Refresh camera list to update the interface with proper labels
                    await this.scanAvailableCameras();
                }
            }

            // Choose appropriate method based on browser support
            if (!this.isSafari && typeof MediaStreamTrackProcessor === 'function') {
                // Use MediaStreamTrackProcessor for Chrome/Android
                this.startCameraWithTrackProcessor(stream, ws);
            } else {
                // Use fallback for Safari/iOS
                this.startCameraWithFallback(stream, ws);
            }

            this.cameraStarted = true;
        } catch (err) {
            console.error('Camera error:', err);
            alert('Camera access failed: ' + err.message);
        }
    }

    // Method for Chrome/Android using MediaStreamTrackProcessor
    startCameraWithTrackProcessor(stream, ws) {
        console.log('Using MediaStreamTrackProcessor for camera streaming');
        
        // Ensure only one processor is running at a time
        if (this._processorRunning) {
            console.log('TrackProcessor already running, not starting another one');
            return;
        }
        
        try {
            const trackProcessor = new MediaStreamTrackProcessor({ track: this.videoTrack });
            const reader = trackProcessor.readable.getReader();

            // Use consistent dimensions
            console.log(`Creating OffscreenCanvas with dimensions: ${this.fixedWidth}x${this.fixedHeight}`);
            const canvas = new OffscreenCanvas(this.fixedWidth, this.fixedHeight);
            const ctx = canvas.getContext('2d');

            let lastSentTime = 0;
            const desiredFps = Math.min(this.cameraConfig.fps, 30);
            const frameInterval = 1000 / desiredFps;
            
            // Mark processor as running
            this._processorRunning = true;

            const processFrame = async (videoFrame) => {
                try {
                    const bitmap = await createImageBitmap(videoFrame);
                    
                    // Scale to fit canvas while maintaining aspect ratio
                    const scale = Math.min(canvas.width / bitmap.width, canvas.height / bitmap.height);
                    const x = (canvas.width - bitmap.width * scale) / 2;
                    const y = (canvas.height - bitmap.height * scale) / 2;
                    
                    // Clear canvas and draw scaled image
                    ctx.clearRect(0, 0, canvas.width, canvas.height);
                    ctx.drawImage(bitmap, x, y, bitmap.width * scale, bitmap.height * scale);
                    
                    // Clean up the bitmap after use
                    bitmap.close();
                    
                    // Convert to JPEG blob using quality from config
                    const blob = await canvas.convertToBlob({
                        type: 'image/jpeg',
                        quality: this.cameraConfig.quality
                    });
                    
                    // Convert blob to base64
                    return new Promise((resolve, reject) => {
                        const reader = new FileReader();
                        reader.onloadend = () => resolve(reader.result);
                        reader.onerror = reject;
                        reader.readAsDataURL(blob);
                    });
                } catch (error) {
                    console.error('Error processing frame:', error);
                    throw error;
                }
            };

            const processFrames = async () => {
                while (this._processorRunning) {
                    try {
                        const { done, value: videoFrame } = await reader.read();
                        if (done || !this._processorRunning) break;

                        const currentTime = performance.now();
                        if (currentTime - lastSentTime < frameInterval) {
                            videoFrame.close();
                            continue;
                        }

                        if (ws && ws.readyState === WebSocket.OPEN) {
                            try {
                                this.lastCameraFrame = await processFrame(videoFrame);
                                
                                ws.send(JSON.stringify({
                                    timestamp: Date.now(),
                                    camera: this.lastCameraFrame,
                                    width: this.fixedWidth,
                                    height: this.fixedHeight
                                }));
                                lastSentTime = currentTime;
                            } catch (err) {
                                console.error('Frame processing error:', err);
                            }
                        }
                        videoFrame.close();
                    } catch (e) {
                        console.error('Error reading video frame:', e);
                        if (this._processorRunning) {
                            // Only break if we're still meant to be running
                            break;
                        }
                    }
                }
                
                console.log('Track processor stopped');
                this._processorRunning = false;
            };

            processFrames().catch(e => {
                console.error('Process frames loop error:', e);
                this._processorRunning = false;
            });
            
        } catch (err) {
            console.error('Error with TrackProcessor, falling back to compatibility mode:', err);
            this._processorRunning = false;
            // If TrackProcessor fails, fall back to the compatibility method
            this.startCameraWithFallback(stream, ws);
        }
    }

    // Fallback method for Safari/iOS using video+canvas approach
    startCameraWithFallback(stream, ws) {
        console.log('Using fallback method for camera streaming (Safari/iOS)');
        
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
        
        // For iOS Safari, we need to ensure video loads properly
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
                    console.log('Video playback started for fallback camera method');
                    
                    // For iOS Safari, we need to wait a bit to make sure video is actually playing
                    if (this.isSafari) {
                        setTimeout(startCapturing, 500);
                    } else {
                        startCapturing();
                    }
                })
                .catch(err => {
                    console.error('Video playback failed:', err);
                    
                    // For iOS, video play must be initiated by user gesture
                    if (this.isSafari) {
                        console.log('Attempting alternative play method for iOS');
                        
                        // Create a temporary play button (iOS requires user interaction)
                        const playButton = document.createElement('button');
                        playButton.textContent = 'Start Camera';
                        playButton.style.position = 'fixed';
                        playButton.style.top = '50%';
                        playButton.style.left = '50%';
                        playButton.style.transform = 'translate(-50%, -50%)';
                        playButton.style.zIndex = '9999';
                        playButton.style.padding = '15px 30px';
                        playButton.style.fontSize = '18px';
                        document.body.appendChild(playButton);
                        
                        playButton.addEventListener('click', () => {
                            this.videoElement.play()
                                .then(() => {
                                    console.log('Video playback started after user interaction');
                                    startCapturing();
                                    document.body.removeChild(playButton);
                                })
                                .catch(playErr => {
                                    console.error('Video play failed even after user interaction:', playErr);
                                    document.body.removeChild(playButton);
                                });
                        });
                    }
                });
        } else {
            console.log('Play promise not supported, starting capture immediately');
            startCapturing();
        }
    }

    stopCamera() {
        console.log('Stopping camera and cleaning up resources');
        
        // Stop track processor
        this._processorRunning = false;
        
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
        console.log('Camera fully stopped');
    }

    getLastFrame() {
        const frame = this.lastCameraFrame;
        this.lastCameraFrame = null;
        return frame;
    }
}

// Export the camera manager
window.CameraManager = CameraManager;

// Initialize camera scanning when the page loads
document.addEventListener('DOMContentLoaded', () => {
    if (window.CameraManager) {
        const cameraManager = new CameraManager();
        // Expose camera manager globally for the camera selection interface
        window.cameraManager = cameraManager;
        // Pre-scan cameras
        cameraManager.scanAvailableCameras().catch(console.error);
    }
});
