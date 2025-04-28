// Global variables and state management
let xrSession = null;
let xrRefSpace = null;
const xrButton = document.getElementById('xr-button');
const poseDiv = document.getElementById('pose');
let poseWs = null;  // Separate WebSocket for pose data
let cameraWs = null;  // Separate WebSocket for camera data
let ttsWs = null;  // Reference to TTS WebSocket
let isSessionActive = false;
let microphoneWs = null;
let cameraInterval = null;
let cameraSendingActive = false;

// Add sensor selection states
let enabledSensors = {
  camera: true,
  pose: true,
  microphone: true,
  audio: true
};

// Initialize when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
  // Initialize UI and add event listeners
  initializeUI();
  
  // Load audio configuration
  loadAudioScript().then(audioConfig => {
    if (window.TextToSpeech && (!window.tts || !window.tts.isReady)) {
      window.tts = new TextToSpeech();
    }
  });
  
  // Populate camera dropdown
  populateCameraDropdown();
  
  // Check if XR is supported
  checkSupported();
});

// Function to initialize UI and attach event listeners
function initializeUI() {
  // Check sensor data
  const cameraManager = new CameraManager();
  const speechRecognitionManager = new SpeechRecognitionManager();
  window.cameraManager = cameraManager;
  window.speechRecognitionManager = speechRecognitionManager;
  
  // Add checkbox event listeners
  document.getElementById('camera-select').addEventListener('change', (e) => {
    enabledSensors.camera = e.target.checked;
    document.getElementById('camera-select-container').style.display = e.target.checked ? 'flex' : 'none';
    if (!e.target.checked && cameraInterval) {
      stopCameraSending();
    } else if (e.target.checked && isSessionActive) {
      startCameraSending();
    }
  });

  document.getElementById('pose-select').addEventListener('change', (e) => {
    enabledSensors.pose = e.target.checked;
    poseDiv.style.display = e.target.checked ? 'block' : 'none';
    checkSupported(); // Recheck AR support when pose tracking is toggled
  });

  document.getElementById('microphone-select').addEventListener('change', (e) => {
    enabledSensors.microphone = e.target.checked;
    document.getElementById('transcription-log').style.display = e.target.checked ? 'block' : 'none';
    if (!e.target.checked) {
      speechRecognitionManager.stopSpeechRecognition();
    } else if (isSessionActive) {
      connectWebSockets();
    }
  });

  // Add Audio checkbox listener - this refers to output audio
  document.getElementById('audio-select').addEventListener('change', async (e) => {
    enabledSensors.audio = e.target.checked;
    
    if (!e.target.checked) {
      if (window.tts) {
        window.tts.disconnectWebSocket();
      }
      updateConnectionStatus('audio', 'disconnected');
    } else if (e.target.checked && isSessionActive) {
      updateConnectionStatus('audio', 'connecting');
      
      if (!window.tts) {
        window.tts = new TextToSpeech();
      }
      
      try {
        await window.tts.connectWebSocket();
        updateConnectionStatus('audio', 'connected');
      } catch (error) {
        console.error('Failed to connect TTS WebSocket:', error);
        updateConnectionStatus('audio', 'disconnected');
      }
    }
  });
  
  // Add XR button event listener
  xrButton.addEventListener('click', () => {
    if (!isSessionActive) {
      startSession();
    } else {
      if (xrSession) {
        xrSession.end();
      } else {
        // For non-AR sessions, call onSessionEnded directly
        onSessionEnded();
      }
    }
  });
}

// Function to populate camera dropdown
async function populateCameraDropdown() {
  try {
    const dropdown = document.getElementById('camera-dropdown');
    if (!dropdown) return;
    
    // Clear existing options
    dropdown.innerHTML = '';
    
    // Get available cameras
    const devices = await navigator.mediaDevices.enumerateDevices();
    const videoDevices = devices.filter(device => device.kind === 'videoinput');
    
    if (videoDevices.length === 0) {
      const option = document.createElement('option');
      option.text = 'No cameras found';
      dropdown.add(option);
      return;
    }
    
    // Add options for each camera
    videoDevices.forEach((device, index) => {
      const option = document.createElement('option');
      option.value = device.deviceId;
      option.text = device.label || `Camera ${index + 1}`;
      dropdown.add(option);
    });
    
    // Add event listener to handle camera change
    dropdown.addEventListener('change', (e) => {
      if (window.cameraManager && typeof window.cameraManager.switchCamera === 'function') {
        window.cameraManager.switchCamera(e.target.value);
      }
    });
    
    // Show container if we have cameras
    document.getElementById('camera-select-container').style.display = videoDevices.length > 0 ? 'flex' : 'none';
  } catch (error) {
    console.error('Error populating camera dropdown:', error);
  }
}

// Function to dynamically load audio script based on config
async function loadAudioScript() {
  try {
    const response = await fetch('/api/config');
    const config = await response.json();
    return config.audio || { mode: 'tts', enabled: true };
  } catch (error) {
    console.error('Error loading audio configuration:', error);
    return { mode: 'tts', enabled: true };
  }
}

// Helper function to dynamically load scripts
function loadScript(src) {
  return new Promise((resolve, reject) => {
    const script = document.createElement('script');
    script.src = src;
    script.onload = resolve;
    script.onerror = reject;
    document.head.appendChild(script);
  });
}

// Function to check if XR is supported
function checkSupported() {
  if (!enabledSensors.pose) {
    xrButton.disabled = false;
    xrButton.textContent = 'Start';
    return;
  }

  if (navigator.xr) {
    navigator.xr.isSessionSupported('immersive-ar').then(supported => {
      xrButton.disabled = !supported;
      xrButton.textContent = supported ? 'Start' : 'XR not found';
    });
  } else {
    xrButton.disabled = true;
    xrButton.textContent = 'XR not supported';
  }
}

// Function to update connection status indicators
function updateConnectionStatus(type, status) {
  const statusEl = document.getElementById(`${type}-status`);
  if (statusEl) {
    statusEl.className = `connection-status ${status}`;
  }
  
  // Log status changes to the console
  if (status === 'disconnected') {
    console.log(`${type.charAt(0).toUpperCase() + type.slice(1)} sensor disconnected`);
  } else if (status === 'connected') {
    console.log(`${type.charAt(0).toUpperCase() + type.slice(1)} sensor connected`);
  } else if (status === 'connecting') {
    console.log(`${type.charAt(0).toUpperCase() + type.slice(1)} sensor connecting...`);
  }
}

// Connect WebSockets for all enabled sensors
function connectWebSockets() {
  if (!isSessionActive) return;
  
  console.log('Connecting WebSockets for enabled sensors...');
  
  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  const baseUrl = `${protocol}//${window.location.host}`;
  
  if (enabledSensors.pose) {
    connectPoseWebSocket(baseUrl);
  }

  if (enabledSensors.camera) {
    connectCameraWebSocket(baseUrl);
  }

  if (enabledSensors.microphone) {
    connectMicrophoneWebSocket(baseUrl);
  }

  // Connect Audio WebSocket (output)
  if (enabledSensors.audio) {
    updateConnectionStatus('audio', 'connecting');
    
    if (!window.tts) {
      window.tts = new TextToSpeech();
    }
    
    window.tts.connectWebSocket().then(() => {
      console.log('Audio WebSocket connected successfully');
      updateConnectionStatus('audio', 'connected');
      ttsWs = window.tts.ws;
    }).catch(err => {
      console.error('Failed to connect Audio WebSocket:', err);
      updateConnectionStatus('audio', 'disconnected');
    });
    
    // Connect WAV audio WebSocket if enabled
    fetch('/api/config')
      .then(response => response.json())
      .then(config => {
        if (config.audio && config.audio.mode === 'wav' && config.audio.enabled) {
          console.log('WAV audio mode detected, activating WAV WebSocket...');
          if (window.audioPlayer && typeof window.audioPlayer.connectWebSocket === 'function') {
            console.log('WAV audio player connected and activated');
          } else {
            console.log('WAV audio player not available or not fully initialized');
          }
        }
      })
      .catch(err => console.error('Failed to check WAV audio config:', err));
  }
}

// Function to connect pose WebSocket
function connectPoseWebSocket(baseUrl) {
  updateConnectionStatus('pose', 'connecting');
  
  // Connect pose WebSocket
  poseWs = new WebSocket(`${baseUrl}/pose`);
  poseWs.onopen = () => {
    console.log('Pose WebSocket connected');
    updateConnectionStatus('pose', 'connected');
  };
  poseWs.onerror = (error) => {
    console.error('Pose WebSocket error:', error);
    updateConnectionStatus('pose', 'disconnected');
  };
  poseWs.onclose = () => {
    updateConnectionStatus('pose', 'disconnected');
    if (isSessionActive) {
      setTimeout(() => {
        if (enabledSensors.pose && isSessionActive) {
          connectPoseWebSocket(baseUrl);
        }
      }, 1000);
    }
  };
}

// Function to connect camera WebSocket
function connectCameraWebSocket(baseUrl) {
  updateConnectionStatus('camera', 'connecting');
  
  // Connect camera WebSocket
  cameraWs = new WebSocket(`${baseUrl}/camera`);
  window.cameraWs = cameraWs;
  
  cameraWs.onopen = () => {
    console.log('Camera WebSocket connected');
    updateConnectionStatus('camera', 'connected');
    if (!cameraInterval && isSessionActive) {
      startCameraSending();
    }
  };
  cameraWs.onerror = (error) => {
    console.error('Camera WebSocket error:', error);
    updateConnectionStatus('camera', 'disconnected');
  };
  cameraWs.onclose = () => {
    updateConnectionStatus('camera', 'disconnected');
    window.cameraWs = null;
    if (isSessionActive) {
      setTimeout(() => {
        if (enabledSensors.camera && isSessionActive) {
          connectCameraWebSocket(baseUrl);
        }
      }, 1000);
    }
  };
}

// Function to connect microphone WebSocket
function connectMicrophoneWebSocket(baseUrl) {
  updateConnectionStatus('microphone', 'connecting');
  
  microphoneWs = new WebSocket(`${baseUrl}/microphone`);
  microphoneWs.onopen = () => {
    console.log('Microphone WebSocket connected');
    updateConnectionStatus('microphone', 'connected');
    window.speechRecognitionManager.startSpeechRecognition(microphoneWs, isSessionActive);
  };
  microphoneWs.onerror = (error) => {
    console.error('Microphone WebSocket error:', error);
    updateConnectionStatus('microphone', 'disconnected');
  };
  microphoneWs.onclose = () => {
    updateConnectionStatus('microphone', 'disconnected');
    if (isSessionActive) {
      setTimeout(() => {
        if (enabledSensors.microphone && isSessionActive) {
          connectMicrophoneWebSocket(baseUrl);
        }
      }, 1000);
    }
  };
  
  // Add message handler for transcription results
  microphoneWs.onmessage = (event) => {
    try {
      const data = JSON.parse(event.data);
      if (data.transcription) {
        addTranscriptionEntry(data.transcription);
      }
    } catch (error) {
      console.error('Error parsing microphone response:', error);
    }
  };
}

// Function to start camera frame sending
function startCameraSending() {
  if (cameraSendingActive) return;
  
  cameraSendingActive = true;
  cameraInterval = setInterval(() => {
    if (cameraWs && cameraWs.readyState === WebSocket.OPEN) {
      const cameraFrame = window.cameraManager.getLastFrame();
      if (cameraFrame) {
        cameraWs.send(JSON.stringify({
          timestamp: Date.now(),
          camera: cameraFrame,
          width: window.cameraManager.fixedWidth || 480,
          height: window.cameraManager.fixedHeight || 640
        }));
      }
    }
  }, 30);
}

// Function to stop camera frame sending
function stopCameraSending() {
  cameraSendingActive = false;
  if (cameraInterval) {
    clearInterval(cameraInterval);
    cameraInterval = null;
  }
}

// Function to start the AR/XR session
async function startSession() {
  try {
    if (enabledSensors.pose) {
      // Start AR session only if position tracking is enabled
      xrSession = await navigator.xr.requestSession('immersive-ar', {
        requiredFeatures: ['local'],
        optionalFeatures: ['dom-overlay'],
        domOverlay: { root: document.body },
        environmentBlendMode: 'opaque'
      });
      xrSession.addEventListener('end', onSessionEnded);

      const canvas = document.createElement('canvas');
      const gl = canvas.getContext('webgl', { xrCompatible: true });
      await xrSession.updateRenderState({
        baseLayer: new XRWebGLLayer(xrSession, gl)
      });

      xrRefSpace = await xrSession.requestReferenceSpace('local');
      xrSession.requestAnimationFrame(onXRFrame);
    }

    isSessionActive = true;
    xrButton.textContent = 'Stop';
    
    // Connect WebSockets first
    connectWebSockets();
    
    if (enabledSensors.camera) {
      window.cameraManager.startCamera(cameraWs, isSessionActive);
    }

    document.body.classList.add('session-active');

  } catch (e) {
    console.error('Failed to start session:', e);
    onSessionEnded();  // Clean up if start fails
  }
}

// When the session ends, reset button and display
function onSessionEnded() {
  if (!isSessionActive) return;  // Prevent multiple calls
  
  // Reset all connection statuses
  ['pose', 'camera', 'microphone', 'audio'].forEach(type => {
    updateConnectionStatus(type, '');
  });
  
  isSessionActive = false;
  document.body.classList.remove('session-active');
  
  // Clean up WebSockets
  [poseWs, cameraWs, microphoneWs].forEach(ws => {
    if (ws) {
      ws.close();
    }
  });
  poseWs = cameraWs = microphoneWs = null;

  // Clean up managers
  window.cameraManager.stopCamera();
  window.speechRecognitionManager.stopSpeechRecognition();
  stopCameraSending();
  
  // Properly disconnect Audio
  if (window.tts) {
    window.tts.disconnectWebSocket();
  }
  
  // Disconnect WAV audio player if it exists
  if (window.audioPlayer && typeof window.audioPlayer.disconnectWebSocket === 'function') {
    window.audioPlayer.disconnectWebSocket();
    window.audioPlayer.setSessionActive(false);
    console.log('WAV audio player disconnected');
  }
  
  // Reset XR session
  if (xrSession) {
    xrSession = null;
    xrRefSpace = null;
  }

  // Update UI
  xrButton.textContent = 'Start';
  poseDiv.textContent = 'Session ended.';
}

// Handle XR animation frame updates
function onXRFrame(time, frame) {
  xrSession.requestAnimationFrame(onXRFrame);
  const pose = frame.getViewerPose(xrRefSpace);
  if (pose && enabledSensors.pose) {
    const pos = pose.transform.position;
    const o = pose.transform.orientation;
    const text = `Position: ${pos.x.toFixed(3)}, ${pos.y.toFixed(3)}, ${pos.z.toFixed(3)}
      Orientation: ${o.x.toFixed(3)}, ${o.y.toFixed(3)}, ${o.z.toFixed(3)}, ${o.w.toFixed(3)}`;
    // Print on mobile screen
    poseDiv.textContent = text;
    
    // Send pose data via dedicated WebSocket
    if (poseWs && poseWs.readyState === WebSocket.OPEN) {
      const poseData = {
        timestamp: Date.now(),
        pose: {
          position: { x: pos.x, y: pos.y, z: pos.z },
          orientation: { x: o.x, y: o.y, z: o.z, w: o.w }
        }
      };
      poseWs.send(JSON.stringify(poseData));
    }
  } else {
    poseDiv.textContent = 'No pose available.';
  }
}

// Function to add transcription entries to the log
function addTranscriptionEntry(text) {
  const transcriptionLog = document.getElementById('transcription-log');
  if (!transcriptionLog) return;
  
  const entry = document.createElement('div');
  entry.className = 'log-entry';
  
  const timestamp = document.createElement('span');
  timestamp.className = 'timestamp';
  timestamp.textContent = new Date().toLocaleTimeString();
  
  const content = document.createElement('span');
  content.textContent = text;
  
  entry.appendChild(timestamp);
  entry.appendChild(content);
  transcriptionLog.appendChild(entry);
  
  // Auto-scroll to the latest entry
  transcriptionLog.scrollTop = transcriptionLog.scrollHeight;
  
  // Limit the number of entries
  while (transcriptionLog.children.length > 50) {
    transcriptionLog.removeChild(transcriptionLog.firstChild);
  }
}