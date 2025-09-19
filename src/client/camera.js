class CameraManager {
    constructor() {
        this.cameraStarted = false;
        this.videoTrack = null;
        this.lastCameraFrame = null;
        this.cameraConfig = {
            facingMode: "environment", // Default value until config is loaded
            fps: 15, // Default FPS
            quality: 0.7 // Default quality
        };
        
        // Fetch camera configuration when created
        this.fetchCameraConfig();
    }
    
    // New method to fetch camera configuration from the server
    async fetchCameraConfig() {
        try {
            const response = await fetch('/api/config');
            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }
            const config = await response.json();
            if (config.camera) {
                if (config.camera.facingMode) {
                    this.cameraConfig.facingMode = config.camera.facingMode;
                    console.log('Using camera facing mode from config:', this.cameraConfig.facingMode);
                }
                if (config.camera.fps) {
                    this.cameraConfig.fps = config.camera.fps;
                    console.log('Using camera FPS from config:', this.cameraConfig.fps);
                }
                if (config.camera.quality) {
                    this.cameraConfig.quality = config.camera.quality;
                    console.log('Using camera quality from config:', this.cameraConfig.quality);
                }
            }
        } catch (error) {
            console.error('Failed to load camera config:', error);
            // Keep using the defaults
        }
    }

    // Add toggle camera method to switch between front and back cameras
    async toggleCamera() {
        // If camera is running, stop it first
        if (this.cameraStarted && this.videoTrack) {
            this.videoTrack.stop();
            this.videoTrack = null;
        }
        
        // Toggle the facing mode
        this.cameraConfig.facingMode = 
            this.cameraConfig.facingMode === "environment" ? "user" : "environment";
        
        console.log(`Camera toggled to: ${this.cameraConfig.facingMode}`);
        
        // Restart the camera with new facing mode if it was running
        if (this.cameraStarted && window.cameraWs) {
            // Get the session active state from window object, not global
            const isActive = window.isSessionActive !== undefined ? window.isSessionActive : true;
            await this.startCamera(window.cameraWs, isActive);
        }
    }

    async startCamera(ws, isSessionActive) {
        if (this.cameraStarted || !isSessionActive) return;
        try {
            if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
                throw new Error('getUserMedia API not supported');
            }

            // Check if 3D Position is enabled and override to user facing if it is
            const poseEnabled = document.getElementById('pose-select').checked;
            const facingMode = poseEnabled ? "user" : this.cameraConfig.facingMode;
            console.log(`Starting camera with facing mode: ${facingMode} (pose enabled: ${poseEnabled})`);

            const stream = await navigator.mediaDevices.getUserMedia({
                video: { facingMode: facingMode },
            });
            this.videoTrack = stream.getVideoTracks()[0];

            const trackProcessor = new MediaStreamTrackProcessor({ track: this.videoTrack });
            const reader = trackProcessor.readable.getReader();

            // Create canvas once for reuse
            const canvas = new OffscreenCanvas(640, 480); // Fixed size for performance
            const ctx = canvas.getContext('2d');

            let lastSentTime = 0;
            const desiredFps = this.cameraConfig.fps; // Use FPS from config
            const frameInterval = 1000 / desiredFps;
            const imageQuality = this.cameraConfig.quality; // Capture quality from config

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
                    quality: imageQuality // Use quality from config
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