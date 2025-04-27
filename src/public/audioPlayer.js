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
  
  // Create and inject the audio element into the DOM
  function createAudioElement() {
    if (!audioElement) {
      audioElement = document.createElement('audio');
      audioElement.id = 'wav-audio-player';
      audioElement.style.display = 'none';
      audioElement.controls = false;
      audioElement.autoplay = true;
      
      // Add event listeners for better status reporting
      audioElement.onplay = () => {
        console.log('Audio playback started');
        isPlaying = true;
        updateWavStatus('Playing', 'HTML5 Audio');
      };
      
      audioElement.onended = () => {
        console.log('Audio playback completed');
        isPlaying = false;
        updateWavStatus('Ready', 'Waiting for next audio');
        
        // Play next item in queue if available
        if (audioQueue.length > 0) {
          console.log(`Playing next audio from queue (${audioQueue.length} remaining)`);
          const nextAudio = audioQueue.shift();
          playAudioData(nextAudio);
          updateWavStatus('Playing', `Queued audio (${audioQueue.length} remaining)`);
        }
      };
      
      audioElement.onerror = (e) => {
        console.error('Audio playback error:', audioElement.error);
        isPlaying = false;
        updateWavStatus('Error', `Code: ${audioElement.error ? audioElement.error.code : 'unknown'}`);
        
        // Try to play next item in queue even if there was an error
        if (audioQueue.length > 0) {
          console.log(`Attempting next audio after error (${audioQueue.length} remaining)`);
          const nextAudio = audioQueue.shift();
          setTimeout(() => playAudioData(nextAudio), 500); // Small delay before trying next audio
        }
      };
      
      // Append to document body
      document.body.appendChild(audioElement);
      console.log('Created persistent audio element');
      
      // Try to unlock audio on iOS/Safari with first user interaction
      document.addEventListener('click', () => {
        if (audioElement) {
          // Try playing silence to unlock audio
          const silenceDataURL = 'data:audio/wav;base64,UklGRigAAABXQVZFZm10IBIAAAABAAEARKwAAIhYAQACABAAAABkYXRhAgAAAAEA';
          audioElement.src = silenceDataURL;
          audioElement.play().catch(e => console.warn('Silent audio play failed:', e));
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
      console.log('Audio configuration loaded:', audioConfig);
      
      // Only connect if audio is enabled and mode is set to wav
      if (audioConfig.enabled && audioConfig.mode === 'wav') {
        // Create the audio element
        createAudioElement();
        connectWebSocket();
        return true;
      } else {
        console.log('WAV audio player disabled by configuration');
        return false;
      }
    } catch (error) {
      console.error('Error loading configuration:', error);
      return false;
    }
  }
  
  // Helper function to update the UI connection status if available
  function updateUIStatus(status) {
    if (window.updateConnectionStatus) {
      window.updateConnectionStatus('tts', status);
    }
  }
  
  // Helper function to update the WAV audio status text
  function updateWavStatus(status, details = '') {
    const statusElement = document.getElementById('wav-status-text');
    if (statusElement) {
      let message = status;
      if (details) {
        message += ` (${details})`;
      }
      statusElement.textContent = message;
      
      // Set color based on status
      if (status.includes('Error') || status.includes('Failed')) {
        statusElement.style.color = '#ff4d4d'; // Red for errors
      } else if (status.includes('Connected') || status.includes('Playing')) {
        statusElement.style.color = '#4CAF50'; // Green for success
      } else if (status.includes('Connecting') || status.includes('Waiting')) {
        statusElement.style.color = '#ff9800'; // Orange for in-progress
      } else {
        statusElement.style.color = '#999'; // Default gray
      }
    }
  }
  
  // Connect to WebSocket for WAV audio
  function connectWebSocket() {
    if (audioConfig.mode !== 'wav' || !audioConfig.enabled) {
      console.log('WAV audio player not connecting - disabled by configuration');
      updateWavStatus('Disabled by configuration');
      return;
    }
    
    if (ws && (ws.readyState === WebSocket.CONNECTING || ws.readyState === WebSocket.OPEN)) {
      console.log('WebSocket connection already exists');
      return;
    }
    
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const baseUrl = `${protocol}//${window.location.host}`;
    
    // Update UI status to connecting
    updateUIStatus('connecting');
    updateWavStatus('Connecting...');
    
    console.log('Connecting to WAV audio WebSocket at:', `${baseUrl}/wav_audio`);
    ws = new WebSocket(`${baseUrl}/wav_audio`);
    ws.binaryType = 'arraybuffer'; // Important for binary data
    
    ws.onmessage = function(event) {
      // Handle text/JSON messages
      if (typeof event.data === 'string') {
        try {
          const jsonData = JSON.parse(event.data);
          console.log('Received JSON message:', jsonData);
          // Check if this is a session state message
          if (jsonData.sessionState !== undefined) {
            isSessionActive = jsonData.sessionState === 'active';
            console.log(`Session state updated to: ${isSessionActive ? 'active' : 'inactive'}`);
            updateWavStatus(isSessionActive ? 'Ready' : 'Waiting', 
                           isSessionActive ? 'Waiting for audio' : 'Session not started');
            return;
          }
          updateWavStatus('Ready', 'Waiting for audio');
        } catch (e) {
          console.log('Received string message:', event.data);
          updateWavStatus('Received message', event.data.substring(0, 20));
        }
        return;
      }
      
      // Handle binary audio data
      console.log('Received audio data', event.data.byteLength, 'bytes');
      
      // Only process audio data if session is active
      if (!isSessionActive) {
        console.log('Audio received but session not active, ignoring');
        updateWavStatus('Waiting', 'Session not active');
        return;
      }
      
      updateWavStatus('Processing audio', `${event.data.byteLength} bytes`);
      
      // Ensure the audio element exists
      createAudioElement();
      
      // Process the audio data
      processAudioData(event.data);
    };
    
    ws.onopen = () => {
      console.log('Connected to WAV audio stream server');
      updateUIStatus('connected');
      updateWavStatus('Connected', 'Waiting for audio');
    };
    
    ws.onclose = () => {
      console.log('Audio stream connection closed');
      updateUIStatus('disconnected');
      updateWavStatus('Disconnected');
      
      // Attempt to reconnect after a delay
      if (autoReconnect) {
        console.log('Attempting to reconnect...');
        setTimeout(connectWebSocket, 3000);
      }
    };
    
    ws.onerror = (error) => {
      console.error('WebSocket error:', error);
      updateUIStatus('disconnected');
      updateWavStatus('Error', 'Connection failed');
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
        console.log("Adding WAV header to audio data");
        processedData = addWavHeader(audioData);
      } else {
        console.log("Using existing WAV header from audio data");
      }
    }
    
    // If audio is currently playing, add to queue
    if (isPlaying) {
      audioQueue.push(processedData);
      console.log(`Audio added to queue. Queue length: ${audioQueue.length}`);
      updateWavStatus('Queued', `${audioQueue.length} items waiting`);
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
      .then(() => {
        console.log('Audio playback started successfully');
      })
      .catch(err => {
        console.error('Error starting audio playback:', err);
        isPlaying = false;
        
        if (err.name === 'NotAllowedError') {
          updateWavStatus('Blocked', 'Tap screen once to enable audio');
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
        audioElement.play().catch(e => console.warn('Audio play failed after click:', e));
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
    console.log('WAV audio player initializing');
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
              audioElement.play().catch(e => console.warn('Silent audio play failed:', e));
            }
          } else {
            disconnectWebSocket();
          }
        }
      });
    }
    
    // Connect the audio unlock button if it exists
    const unlockBtn = document.getElementById('audio-unlock-button');
    if (unlockBtn) {
      unlockBtn.addEventListener('click', () => {
        console.log('Audio unlock button clicked');
        if (audioElement) {
          const silenceDataURL = 'data:audio/wav;base64,UklGRigAAABXQVZFZm10IBIAAAABAAEARKwAAIhYAQACABAAAABkYXRhAgAAAAEA';
          audioElement.src = silenceDataURL;
          audioElement.play().catch(e => console.warn('Silent audio play failed after button click:', e));
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
          console.warn('Manual audio unlock failed:', e);
          return false;
        });
      }
      return Promise.reject(new Error('No audio element available'));
    },
    hasAudioElement: () => !!audioElement,
    
    // Add new methods for the queue
    clearQueue: () => {
      audioQueue = [];
      console.log('Audio queue cleared');
      return true;
    },
    getQueueLength: () => audioQueue.length,
    
    // Add session state control
    setSessionActive: (active) => {
      const wasActive = isSessionActive;
      isSessionActive = active;
      console.log(`Session state changed from ${wasActive ? 'active' : 'inactive'} to ${active ? 'active' : 'inactive'}`);
      
      if (!active) {
        // Clear the queue when session becomes inactive
        audioQueue = [];
        // Stop any current playback
        if (audioElement && !audioElement.paused) {
          audioElement.pause();
          isPlaying = false;
        }
        updateWavStatus('Waiting', 'Session not active');
      } else {
        updateWavStatus('Ready', 'Waiting for audio');
      }
      return true;
    }
  };
})();