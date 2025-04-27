class CameraManager {
    constructor() {
        this.cameraStarted = false;
        this.videoTrack = null;
        this.lastCameraFrame = null;
        this.cameraConfig = {
            facingMode: "environment" // Default value until config is loaded
        };
        this.availableCameras = [];
        this.selectedCameraId = null;
        this.devicePermissionGranted = false;
        
        // Fetch camera configuration when created
        this.fetchCameraConfig();
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
            if (config.camera && config.camera.facingMode) {
                this.cameraConfig.facingMode = config.camera.facingMode;
                console.log('Using camera facing mode from config:', this.cameraConfig.facingMode);
            }
        } catch (error) {
            console.error('Failed to load camera config:', error);
            // Keep using the default
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

            const trackProcessor = new MediaStreamTrackProcessor({ track: this.videoTrack });
            const reader = trackProcessor.readable.getReader();

            // Create canvas once for reuse
            const canvas = new OffscreenCanvas(640, 480); // Fixed size for performance
            const ctx = canvas.getContext('2d');

            let lastSentTime = 0;
            const desiredFps = 30;
            const frameInterval = 1000 / desiredFps;

            async function processFrame(videoFrame) {
                const bitmap = await createImageBitmap(videoFrame);
                
                // Scale to fit canvas while maintaining aspect ratio
                const scale = Math.min(canvas.width / bitmap.width, canvas.height / bitmap.height);
                const x = (canvas.width - bitmap.width * scale) / 2;
                const y = (canvas.height - bitmap.height * scale) / 2;
                
                // Clear canvas and draw scaled image
                ctx.clearRect(0, 0, canvas.width, canvas.height);
                ctx.drawImage(bitmap, x, y, bitmap.width * scale, bitmap.height * scale);
                
                // Convert to JPEG blob
                const blob = await canvas.convertToBlob({
                    type: 'image/jpeg',
                    quality: 0.7
                });
                
                // Convert blob to base64
                const buffer = await blob.arrayBuffer();
                const base64 = btoa(String.fromCharCode(...new Uint8Array(buffer)));
                return `data:image/jpeg;base64,${base64}`;
            }

            const processFrames = async () => {
                while (true) {
                    const { done, value: videoFrame } = await reader.read();
                    if (done) break;

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
                                width: canvas.width,
                                height: canvas.height
                            }));
                            lastSentTime = currentTime;
                        } catch (err) {
                            console.error('Frame processing error:', err);
                        }
                    }
                    videoFrame.close();
                }
            };

            processFrames().catch(console.error);
            this.cameraStarted = true;
        } catch (err) {
            console.error('Camera error:', err);
            alert('Camera access failed: ' + err.message);
        }
    }

    stopCamera() {
        if (this.videoTrack) {
            this.videoTrack.stop();
            this.videoTrack = null;
        }
        this.cameraStarted = false;
        this.lastCameraFrame = null;
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
