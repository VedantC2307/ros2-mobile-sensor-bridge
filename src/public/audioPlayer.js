/**
 * Audio Player - Receives audio data from WebSocket server and plays it on mobile devices
 * Uses configuration from config.yaml to determine whether to enable WAV audio
 * Uses a persistent HTML5 audio element for simple and reliable playback
 */
(function() {
  // Audio playback variables
  let audioConfig = { mode: 'wav', enabled: true }; // Default, will be updated from server config
  
  // WebSocket connection
  let ws = null;
  let autoReconnect = true;
  
  // Persistent audio element
  let audioElement = null;
  
  // Audio queue for sequential playback
  let audioQueue = [];
  let isPlaying = false;
  
  // Session state tracking
  let isSessionActive = false;
  
  // Debug flag - set to false to disable verbose logging
  const DEBUG_MODE = true;

  // Helper function for debug logging
  function debugLog(...args) {
    if (DEBUG_MODE) {
      console.log('[AudioPlayer]', ...args);
    }
  }

  // Create and inject the audio element into the DOM
  function createAudioElement() {
    if (!audioElement) {
      audioElement = document.createElement('audio');
      audioElement.id = 'wav-audio-player';
      audioElement.style.display = 'none';
      audioElement.controls = false;
      audioElement.autoplay = true;
      
      // Add event listeners for audio events
      audioElement.onplay = () => {
        isPlaying = true;
      };
      
      audioElement.onended = () => {
        isPlaying = false;
        
        // Play next item in queue if available
        if (audioQueue.length > 0) {
          const nextAudio = audioQueue.shift();
          playAudioData(nextAudio);
        }
      };
      
      audioElement.onerror = (e) => {
        console.error('Audio playback error:', audioElement.error);
        isPlaying = false;
        
        // Try to play next item in queue even if there was an error
        if (audioQueue.length > 0) {
          const nextAudio = audioQueue.shift();
          setTimeout(() => playAudioData(nextAudio), 500);
        }
      };
      
      // Append to document body
      document.body.appendChild(audioElement);
      
      // Try to unlock audio on iOS/Safari with first user interaction
      document.addEventListener('click', () => {
        if (audioElement) {
          // Try playing silence to unlock audio
          const silenceDataURL = 'data:audio/wav;base64,UklGRigAAABXQVZFZm10IBIAAAABAAEARKwAAIhYAQACABAAAABkYXRhAgAAAAEA';
          audioElement.src = silenceDataURL;
          audioElement.play().catch(e => {});
        }
      }, { once: true });
    }
    
    return audioElement;
  }
  
  // Get configuration from server
  async function fetchConfig() {
    try {
      const response = await fetch('/api/config');
      const config = await response.json();
      audioConfig = config.audio || audioConfig;
      debugLog('Audio configuration loaded:', audioConfig);
      
      // Only connect if audio is enabled and mode is set to wav
      if (audioConfig.enabled && audioConfig.mode === 'wav') {
        createAudioElement();
        connectWebSocket();
        return true;
      }
      return false;
    } catch (error) {
      console.error('Error loading configuration:', error);
      return false;
    }
  }
  
  // Helper function to update the UI connection status
  function updateUIStatus(status) {
    if (window.updateConnectionStatus) {
      window.updateConnectionStatus('tts', status);
    }
  }
  
  // Connect to WebSocket for WAV audio
  function connectWebSocket() {
    if (audioConfig.mode !== 'wav' || !audioConfig.enabled) {
      return;
    }
    
    if (ws && (ws.readyState === WebSocket.CONNECTING || ws.readyState === WebSocket.OPEN)) {
      return;
    }
    
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const baseUrl = `${protocol}//${window.location.host}`;
    const wsUrl = `${baseUrl}/wav_audio`;
    
    // Update UI status to connecting
    updateUIStatus('connecting');
    
    debugLog('Connecting to WAV audio WebSocket');
    ws = new WebSocket(wsUrl);
    ws.binaryType = 'arraybuffer'; // Important for binary data
    
    ws.onmessage = function(event) {
      // Handle text/JSON messages
      if (typeof event.data === 'string') {
        try {
          const jsonData = JSON.parse(event.data);
          // Check if this is a session state message
          if (jsonData.sessionState !== undefined) {
            isSessionActive = jsonData.sessionState === 'active';
          }
        } catch (e) {
          // Non-JSON text message, just log
          debugLog('Received text message:', event.data.substring(0, 20));
        }
        return;
      }
      
      // Handle binary audio data
      debugLog('Received audio binary data', event.data.byteLength, 'bytes');
      
      // Only process audio data if session is active
      if (!isSessionActive) {
        return;
      }
      
      // Ensure the audio element exists
      createAudioElement();
      
      // Process the audio data
      processAudioData(event.data);
    };
    
    ws.onopen = () => {
      debugLog('Connected to WAV audio stream server');
      updateUIStatus('connected');
      
      // Send a message to indicate we're ready to receive audio
      try {
        ws.send(JSON.stringify({ ready: true, client: 'audioPlayer.js' }));
      } catch (err) {
        console.error('Error sending ready message:', err);
      }
    };
    
    ws.onclose = () => {
      updateUIStatus('disconnected');
      
      // Attempt to reconnect after a delay
      if (autoReconnect) {
        setTimeout(connectWebSocket, 3000);
      }
    };
    
    ws.onerror = () => {
      updateUIStatus('disconnected');
    };
  }
  
  // Process received audio data - now with queue support
  function processAudioData(audioData) {
    if (!audioElement) {
      console.error('Audio element not created');
      return;
    }
    
    // Check if we need to add a WAV header
    let processedData = audioData;
    if (audioData.byteLength >= 12) {
      const headerView = new Uint8Array(audioData.slice(0, 12));
      const isRiff = headerView[0] === 82 && headerView[1] === 73 && 
                     headerView[2] === 70 && headerView[3] === 70;
      const isWave = headerView[8] === 87 && headerView[9] === 65 && 
                     headerView[10] === 86 && headerView[11] === 69;
      
      if (!(isRiff && isWave)) {
        processedData = addWavHeader(audioData);
      }
    } else {
      processedData = addWavHeader(audioData);
    }
    
    // If audio is currently playing, add to queue
    if (isPlaying) {
      audioQueue.push(processedData);
      return;
    }
    
    // Otherwise play immediately
    playAudioData(processedData);
  }
  
  // Function to actually play the audio data
  function playAudioData(audioData) {
    if (!audioElement) {
      console.error('Audio element not created');
      return;
    }
    
    // Create a blob and object URL
    const blob = new Blob([audioData], { type: 'audio/wav' });
    const url = URL.createObjectURL(blob);
    
    // Clean up previous URL if it exists
    if (audioElement.src && audioElement.src.startsWith('blob:')) {
      URL.revokeObjectURL(audioElement.src);
    }
    
    // Set new source and play
    audioElement.src = url;
    
    // Attempt to play with error handling
    audioElement.play()
      .catch(err => {
        console.error('Error starting audio playback:', err);
        isPlaying = false;
        
        if (err.name === 'NotAllowedError') {
          showAudioUnblockNotification();
        }
        
        // Try next in queue
        if (audioQueue.length > 0) {
          const nextAudio = audioQueue.shift();
          setTimeout(() => playAudioData(nextAudio), 500);
        }
      });
  }
  
  // Add WAV header to raw PCM data
  function addWavHeader(audioData) {
    // We'll assume 16-bit PCM, 48000 Hz, mono
    const numChannels = 1;
    const sampleRate = 48000;
    const bitsPerSample = 16;
    
    // Calculate sizes
    const dataSize = audioData.byteLength;
    const blockAlign = numChannels * (bitsPerSample / 8);
    const byteRate = sampleRate * blockAlign;
    
    // Create header buffer
    const headerSize = 44;
    const header = new ArrayBuffer(headerSize);
    const view = new DataView(header);
    
    // RIFF chunk descriptor
    view.setUint8(0, 'R'.charCodeAt(0));
    view.setUint8(1, 'I'.charCodeAt(0));
    view.setUint8(2, 'F'.charCodeAt(0));
    view.setUint8(3, 'F'.charCodeAt(0));
    view.setUint32(4, 36 + dataSize, true); // File size - 8
    view.setUint8(8, 'W'.charCodeAt(0));
    view.setUint8(9, 'A'.charCodeAt(0));
    view.setUint8(10, 'V'.charCodeAt(0));
    view.setUint8(11, 'E'.charCodeAt(0));
    
    // fmt sub-chunk
    view.setUint8(12, 'f'.charCodeAt(0));
    view.setUint8(13, 'm'.charCodeAt(0));
    view.setUint8(14, 't'.charCodeAt(0));
    view.setUint8(15, ' '.charCodeAt(0));
    view.setUint32(16, 16, true); // Sub-chunk size
    view.setUint16(20, 1, true); // Audio format (1 = PCM)
    view.setUint16(22, numChannels, true);
    view.setUint32(24, sampleRate, true);
    view.setUint32(28, byteRate, true);
    view.setUint16(32, blockAlign, true);
    view.setUint16(34, bitsPerSample, true);
    
    // data sub-chunk
    view.setUint8(36, 'd'.charCodeAt(0));
    view.setUint8(37, 'a'.charCodeAt(0));
    view.setUint8(38, 't'.charCodeAt(0));
    view.setUint8(39, 'a'.charCodeAt(0));
    view.setUint32(40, dataSize, true); // Data size
    
    // Combine header and data
    const wavBuffer = new Uint8Array(header.byteLength + audioData.byteLength);
    wavBuffer.set(new Uint8Array(header), 0);
    wavBuffer.set(new Uint8Array(audioData), header.byteLength);
    
    return wavBuffer.buffer;
  }
  
  // Helper function to show a notification to the user that they need to interact
  function showAudioUnblockNotification() {
    if (document.getElementById('audio-unblock-notification')) return;
    
    const notification = document.createElement('div');
    notification.id = 'audio-unblock-notification';
    notification.style.position = 'fixed';
    notification.style.top = '20px';
    notification.style.left = '50%';
    notification.style.transform = 'translateX(-50%)';
    notification.style.backgroundColor = 'rgba(0, 0, 0, 0.8)';
    notification.style.color = 'white';
    notification.style.padding = '10px 20px';
    notification.style.borderRadius = '5px';
    notification.style.zIndex = '10000';
    notification.style.textAlign = 'center';
    notification.style.maxWidth = '80%';
    notification.innerHTML = 'Tap anywhere to enable audio playback <br>(Required by your browser)';
    
    notification.addEventListener('click', function() {
      if (audioElement) {
        audioElement.play().catch(e => {});
      }
      this.remove();
    });
    
    document.body.appendChild(notification);
    
    // Auto-hide after 10 seconds
    setTimeout(() => {
      if (notification.parentNode) {
        notification.remove();
      }
    }, 10000);
  }
  
  function disconnectWebSocket() {
    autoReconnect = false;
    if (ws) {
      ws.close();
      ws = null;
    }
  }
  
  // Initialize on page load
  window.addEventListener('DOMContentLoaded', () => {
    createAudioElement();
    fetchConfig();
    
    // Watch for changes in the audio checkbox
    const audioCheckbox = document.getElementById('tts-select');
    if (audioCheckbox) {
      audioCheckbox.addEventListener('change', async (e) => {
        const response = await fetch('/api/config');
        const config = await response.json();
        const currentAudioConfig = config.audio || { mode: 'wav', enabled: true };
        
        // Only respond to checkbox if we're in WAV mode
        if (currentAudioConfig.mode === 'wav') {
          if (e.target.checked) {
            audioConfig.enabled = true;
            if (!ws || ws.readyState !== WebSocket.OPEN) {
              connectWebSocket();
            }
            
            // Try to play a silent sound to unlock audio
            if (audioElement) {
              const silenceDataURL = 'data:audio/wav;base64,UklGRigAAABXQVZFZm10IBIAAAABAAEARKwAAIhYAQACABAAAABkYXRhAgAAAAEA';
              audioElement.src = silenceDataURL;
              audioElement.play().catch(e => {});
            }
          } else {
            disconnectWebSocket();
          }
        }
      });
    }
  });
  
  // Expose public API
  window.audioPlayer = {
    connectWebSocket,
    disconnectWebSocket,
    isAudioUnlocked: () => audioElement && !audioElement.paused,
    getAudioConfig: () => audioConfig,
    playAudio: (arrayBuffer) => {
      if (audioConfig.mode === 'wav' && audioConfig.enabled && audioElement && isSessionActive) {
        processAudioData(arrayBuffer);
        return true;
      }
      return false;
    },
    unlockAudio: () => {
      if (audioElement) {
        const silenceDataURL = 'data:audio/wav;base64,UklGRigAAABXQVZFZm10IBIAAAABAAEARKwAAIhYAQACABAAAABkYXRhAgAAAAEA';
        return audioElement.play().catch(e => {
          return false;
        });
      }
      return Promise.reject(new Error('No audio element available'));
    },
    hasAudioElement: () => !!audioElement,
    clearQueue: () => {
      audioQueue = [];
      return true;
    },
    getQueueLength: () => audioQueue.length,
    setSessionActive: (active) => {
      isSessionActive = active;
      
      if (!active) {
        // Clear the queue when session becomes inactive
        audioQueue = [];
        // Stop any current playback
        if (audioElement && !audioElement.paused) {
          audioElement.pause();
          isPlaying = false;
        }
      }
      return true;
    }
  };
})();