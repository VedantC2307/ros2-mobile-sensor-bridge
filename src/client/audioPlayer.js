/**
 * Enhanced Audio Player - Receives audio data from WebSocket server and plays it
 * Uses configuration from config.yaml to determine whether to enable WAV audio
 * Supports both Android (HTML5 Audio) and iOS (WebAudio API)
 */
(function() {
  // Audio playback variables
  let audioConfig = { mode: 'wav', enabled: true }; // Default, will be updated from server config
  
  // WebSocket connection
  let ws = null;
  let autoReconnect = true;
  
  // Persistent audio elements - use multiple to handle rapid playback better (for Android)
  const audioElements = [];
  const MAX_AUDIO_ELEMENTS = 3;
  let currentAudioIndex = 0;
  
  // Audio queue for sequential playback
  let audioQueue = [];
  let isPlaying = false;
  
  // Session state tracking
  let isSessionActive = false;
  
  // Platform detection
  const isIOS = /iPad|iPhone|iPod/.test(navigator.userAgent) && !window.MSStream;
  
  // WebAudio API variables (for iOS)
  let audioContext = null;
  let isAudioUnlocked = false;
  
  // Add a variable to track active sound sources on iOS
  let activeSources = [];

  // Debug flag - set to false to disable verbose logging
  const DEBUG_MODE = true;

  // Helper function for debug logging
  function debugLog(...args) {
    if (DEBUG_MODE) {
      console.log('[AudioPlayer]', ...args);
    }
  }

  // Create and inject audio elements into the DOM (for Android)
  function createAudioElements() {
    for (let i = 0; i < MAX_AUDIO_ELEMENTS; i++) {
      if (audioElements.length < MAX_AUDIO_ELEMENTS) {
        const audioElement = document.createElement('audio');
        audioElement.id = `wav-audio-player-${i}`;
        audioElement.style.display = 'none';
        audioElement.controls = false;
        audioElement.autoplay = false; // Change to false initially, we'll play manually
        audioElement.preload = 'auto';
        
        // Add event listeners for audio events
        audioElement.onplay = () => {
          isPlaying = true;
        };
        
        audioElement.onended = () => {
          // Mark this audio element as available
          audioElement.dataset.available = 'true';
          
          // Play next item in queue if available
          playNextInQueue();
        };
        
        audioElement.onerror = (e) => {
          console.error('Audio playback error:', audioElement.error);
          // Mark this audio element as available despite error
          audioElement.dataset.available = 'true';
          
          // Try to play next item in queue even if there was an error
          playNextInQueue();
        };
        
        // Mark as available initially
        audioElement.dataset.available = 'true';
        
        // Append to document body
        document.body.appendChild(audioElement);
        audioElements.push(audioElement);
      }
    }
    
    return audioElements;
  }
  
  // Get available audio element for playback
  function getAvailableAudioElement() {
    // First try to find an available element
    for (const audio of audioElements) {
      if (audio.dataset.available === 'true' && (!audio.src || audio.ended || audio.paused)) {
        audio.dataset.available = 'false'; // Mark as in use
        return audio;
      }
    }
    
    // If none available, use round-robin approach
    currentAudioIndex = (currentAudioIndex + 1) % audioElements.length;
    const audio = audioElements[currentAudioIndex];
    
    // Force cleanup of previous usage
    try {
      audio.pause();
      if (audio.src && audio.src.startsWith('blob:')) {
        URL.revokeObjectURL(audio.src);
      }
    } catch (e) {
      // Ignore errors during cleanup
    }
    
    audio.dataset.available = 'false'; // Mark as in use
    return audio;
  }
  
  // Play next audio in queue if available
  function playNextInQueue() {
    // Check if we have more items to play
    if (audioQueue.length > 0) {
      const nextAudio = audioQueue.shift();
      playAudioData(nextAudio);
    } else {
      isPlaying = false; // Nothing more to play
    }
  }
  
  // iOS Audio Unlocking - Enhanced for better iOS compatibility
  async function unlockIOSAudio() {
    if (!isIOS) return true;
    
    debugLog('Attempting to unlock iOS audio...');
    
    try {
      // Try to use the shared audio context from main.js if available
      if (window.sharedAudioContext) {
        audioContext = window.sharedAudioContext;
        debugLog('Using shared audio context from main.js');
      } else {
        // Create a new one if not available
        audioContext = new (window.AudioContext || window.webkitAudioContext)();
        debugLog('Created new audio context for iOS');
      }
      
      // Resume the audio context (important for iOS)
      if (audioContext.state === 'suspended') {
        await audioContext.resume();
        debugLog('Resumed audio context');
      }
      
      // Create and play a brief silent buffer (crucial for unlocking)
      const buffer = audioContext.createBuffer(1, 1, audioContext.sampleRate);
      const source = audioContext.createBufferSource();
      source.buffer = buffer;
      source.connect(audioContext.destination);
      source.start(0);
      
      // Add a special oscillator that runs for a moment - this helps on iOS 13+
      const oscillator = audioContext.createOscillator();
      oscillator.frequency.value = 440; // A4 note
      oscillator.connect(audioContext.createGain()).connect(audioContext.destination);
      oscillator.start(0);
      setTimeout(() => oscillator.stop(), 10); // Just a brief sound
      
      debugLog('iOS audio unlock attempt complete, context state:', audioContext.state);
      isAudioUnlocked = audioContext.state === 'running';
      
      // If we created a new context, make it available to the main.js
      if (!window.sharedAudioContext && isAudioUnlocked) {
        window.sharedAudioContext = audioContext;
        debugLog('Shared newly created audio context with main.js');
      }
      
      return isAudioUnlocked;
    } catch (err) {
      console.error('Failed to unlock iOS audio:', err);
      return false;
    }
  }
  
  // Play audio using WebAudio API (for iOS)
  async function playAudioWithWebAudio(audioData) {
    if (!audioContext) {
      const unlocked = await unlockIOSAudio();
      if (!unlocked) return false;
    }
    
    if (audioContext.state === 'suspended') {
      try {
        await audioContext.resume();
        console.log('Audio context resumed before playback');
      } catch (err) {
        console.error('Failed to resume audio context:', err);
        return false;
      }
    }
    
    try {
      // Force a silent sound to ensure audio is truly unlocked on iOS
      if (isIOS && !isPlaying) {
        const silentBuffer = audioContext.createBuffer(1, 1, audioContext.sampleRate);
        const silentSource = audioContext.createBufferSource();
        silentSource.buffer = silentBuffer;
        silentSource.connect(audioContext.destination);
        silentSource.start(0);
        console.log('Played silent buffer to ensure iOS audio is unlocked');
      }
      
      // Convert ArrayBuffer to AudioBuffer
      const audioBuffer = await decodeAudioData(audioData);
      
      // Create source node
      const source = audioContext.createBufferSource();
      source.buffer = audioBuffer;
      source.connect(audioContext.destination);
      
      // Handle completion for queue processing
      source.onended = () => {
        // Remove this source from the active sources list when it ends naturally
        const index = activeSources.indexOf(source);
        if (index !== -1) {
          activeSources.splice(index, 1);
        }
        
        debugLog('WebAudio source playback ended');
        playNextInQueue();
      };
      
      // Add this source to our active sources list
      activeSources.push(source);
      
      // Start playback
      source.start(0);
      isPlaying = true;
      debugLog('WebAudio playback started');
      
      return true;
    } catch (err) {
      console.error('WebAudio playback error:', err);
      // Continue to next in queue despite error
      setTimeout(playNextInQueue, 100);
      return false;
    }
  }
  
  // Helper function to decode audio data with proper error handling
  function decodeAudioData(audioData) {
    return new Promise((resolve, reject) => {
      // Some older iOS devices don't support the Promise version of decodeAudioData
      // So we use the callback version wrapped in a Promise
      const decodePromise = audioContext.decodeAudioData(
        audioData,
        (decodedData) => resolve(decodedData),
        (err) => reject(err)
      );
      
      // For newer browsers that return a promise from decodeAudioData
      if (decodePromise && decodePromise instanceof Promise) {
        return decodePromise;
      }
    });
  }
  
  // Get configuration from server
  async function fetchConfig() {
    try {
      const response = await fetch('/api/config');
      const config = await response.json();
      audioConfig = config.audio || audioConfig;
      debugLog('Audio configuration loaded:', audioConfig);

      // Backward compatibility: if enabled flag missing, assume true for wav mode
      if (audioConfig.mode === 'wav' && typeof audioConfig.enabled === 'undefined') {
        console.warn('[AUDIO] "enabled" flag missing in config.audio; defaulting to enabled=true');
        audioConfig.enabled = true;
      }
      
      // Only connect if audio is enabled and mode is set to wav
      if (audioConfig.enabled && audioConfig.mode === 'wav') {
        // For Android, create HTML5 audio elements
        if (!isIOS) {
          createAudioElements();
        }
        
        // For iOS, prepare WebAudio API
        if (isIOS) {
          debugLog('iOS detected, will use WebAudio API for playback');
          // We'll unlock audio on first user interaction
          document.body.addEventListener('touchstart', unlockIOSAudio, { once: true });
          document.body.addEventListener('click', unlockIOSAudio, { once: true });
        }
        
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
            debugLog('Session state updated:', isSessionActive);
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
        debugLog('Session not active, ignoring audio data');
        return;
      }
      
      // Process the audio data
      processAudioData(event.data);
    };
    
    ws.onopen = () => {
      debugLog('Connected to WAV audio stream server');
      updateUIStatus('connected');
      
      // Send a message to indicate we're ready to receive audio
      try {
        ws.send(JSON.stringify({ 
          ready: true, 
          client: 'audioPlayer.js', 
          ios: isIOS // Tell the server if we're on iOS
        }));
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
  
  // Process received audio data
  function processAudioData(audioData) {
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
    
    // Handle platform-specific playback approach
    if (isIOS) {
      // iOS: Use WebAudio API
      const currentlyPlaying = isPlaying;
      
      if (currentlyPlaying) {
        audioQueue.push(processedData);
        debugLog('Added to WebAudio queue, queue length:', audioQueue.length);
      } else {
        playAudioWithWebAudio(processedData);
      }
    } else {
      // Android: Use HTML5 Audio (existing implementation)
      if (audioElements.length === 0) {
        console.error('No audio elements created for playback');
        return;
      }
      
      const currentlyPlaying = audioElements.some(audio => 
        audio.dataset.available === 'false' && audio.currentTime > 0 && !audio.paused && !audio.ended
      );
      
      if (currentlyPlaying) {
        audioQueue.push(processedData);
        debugLog('Added to HTML5 Audio queue, queue length:', audioQueue.length);
      } else {
        playAudioData(processedData);
      }
    }
  }
  
  // Function to play audio data with HTML5 Audio (Android)
  function playAudioData(audioData) {
    if (isIOS) {
      return playAudioWithWebAudio(audioData);
    }
    
    // Android approach with HTML5 Audio
    if (audioElements.length === 0) {
      console.error('No audio elements available');
      return;
    }
    
    // Get an available audio element
    const audioElement = getAvailableAudioElement();
    debugLog('Using audio element', audioElement.id);
    
    // Create a blob and object URL
    const blob = new Blob([audioData], { type: 'audio/wav' });
    const url = URL.createObjectURL(blob);
    
    // Clean up previous URL if it exists
    if (audioElement.src && audioElement.src.startsWith('blob:')) {
      URL.revokeObjectURL(audioElement.src);
    }
    
    // Set new source
    audioElement.src = url;
    audioElement.load();
    
    // Attempt to play with error handling
    const playPromise = audioElement.play();
    
    if (playPromise !== undefined) {
      playPromise.then(() => {
        debugLog('Audio playback started successfully');
        isPlaying = true;
      }).catch(err => {
        console.error('Error starting audio playback:', err, 'for URL:', url.substring(0, 30));
        audioElement.dataset.available = 'true'; // Mark as available again
        
        // Try next in queue
        playNextInQueue();
      });
    } else {
      // For older browsers that don't return a promise
      debugLog('Audio play called (legacy mode)');
      isPlaying = true;
    }
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
  
  function disconnectWebSocket() {
    autoReconnect = false;
    if (ws) {
      ws.close();
      ws = null;
    }
  }
  
  // Add a function to completely stop all audio playback
  function stopAllAudio() {
    debugLog('Stopping all audio playback');
    
    // Clear the queue first
    audioQueue = [];
    isPlaying = false;
    
    if (isIOS && audioContext) {
      // For iOS, explicitly stop all tracked audio sources first
      if (activeSources.length > 0) {
        debugLog(`Stopping ${activeSources.length} active audio sources`);
        
        // Create a copy of the array since we'll be modifying it during iteration
        const sourcesToStop = [...activeSources];
        
        // Stop each source
        sourcesToStop.forEach(source => {
          try {
            source.stop(0);
            // Disconnect the source from the audio graph
            source.disconnect();
            
            // Remove from activeSources array
            const index = activeSources.indexOf(source);
            if (index !== -1) {
              activeSources.splice(index, 1);
            }
          } catch (e) {
            // Ignore errors if source is already stopped
            debugLog('Error stopping source:', e.message);
          }
        });
        
        // Clear the sources array
        activeSources = [];
        debugLog('All audio sources stopped and cleared');
      }
      
      // Use a less destructive approach that preserves our audio context
      try {
        // Suspend the context to pause processing
        audioContext.suspend().then(() => {
          debugLog('Audio context suspended');
        }).catch(e => {
          console.error('Failed to suspend audio context:', e);
        });
        
        // Create a silent gain node
        const silentGain = audioContext.createGain();
        silentGain.gain.setValueAtTime(0, audioContext.currentTime);
        silentGain.connect(audioContext.destination);
        
        // Play a brief silent buffer to flush the audio pipeline
        const silentBuffer = audioContext.createBuffer(1, 1024, audioContext.sampleRate);
        const silentSource = audioContext.createBufferSource();
        silentSource.buffer = silentBuffer;
        silentSource.connect(silentGain);
        silentSource.start(0);
        
        // Mark audio as unlocked for future use
        isAudioUnlocked = true;
        
        // We'll resume the context later when needed - no need to create a new one
        debugLog('Audio output silenced but context preserved for future use');
      } catch (err) {
        console.error('Error stopping WebAudio playback:', err);
      }
    } else {
      // For Android HTML5 Audio
      audioElements.forEach(audio => {
        try {
          if (!audio.paused) {
            audio.pause();
          }
          
          // Clear the source
          if (audio.src && audio.src.startsWith('blob:')) {
            URL.revokeObjectURL(audio.src);
            audio.src = '';
          }
          
          // Reset and mark as available
          audio.currentTime = 0;
          audio.dataset.available = 'true';
        } catch (e) {
          // Ignore errors during cleanup
        }
      });
      debugLog('Stopped all HTML5 Audio playback');
    }
    
    return true;
  }

  // Add a new function to safely reinitialize the audio system after stopping
  async function reinitializeAudio() {
    if (!isIOS || !audioContext) return false;
    
    debugLog('Reinitializing iOS audio system...');
    
    try {
      // Resume the existing audio context if it's suspended
      if (audioContext.state === 'suspended') {
        await audioContext.resume();
        debugLog('Resumed existing audio context:', audioContext.state);
      }
      
      // Play a silent sound to ensure the audio system is active again
      const silentBuffer = audioContext.createBuffer(1, 1, audioContext.sampleRate);
      const silentSource = audioContext.createBufferSource();
      silentSource.buffer = silentBuffer;
      silentSource.connect(audioContext.destination);
      silentSource.start(0);
      
      debugLog('Audio system reinitialized successfully');
      isAudioUnlocked = true;
      return true;
    } catch (error) {
      console.error('Failed to reinitialize audio system:', error);
      return false;
    }
  }

  // Initialize on page load
  window.addEventListener('DOMContentLoaded', () => {
    fetchConfig();
    
    // Create audio elements immediately for Android
    if (!isIOS) {
      createAudioElements();
    }
    
    // Watch for changes in the audio checkbox
    const audioCheckbox = document.getElementById('audio-select');
    if (audioCheckbox) {
      audioCheckbox.addEventListener('change', async (e) => {
        try {
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
              
              // For iOS, make sure audio is unlocked
              if (isIOS) {
                unlockIOSAudio();
              }
            } else {
              disconnectWebSocket();
              audioConfig.enabled = false;
            }
          }
        } catch (error) {
          console.error('Error checking audio config:', error);
        }
      });
    }
  });
  
  // Expose public API
  window.audioPlayer = {
    connectWebSocket,
    disconnectWebSocket,
    isAudioUnlocked: () => isIOS ? isAudioUnlocked : true,
    getAudioConfig: () => audioConfig,
    playAudio: (arrayBuffer) => {
      if (audioConfig.mode === 'wav' && audioConfig.enabled && isSessionActive) {
        processAudioData(arrayBuffer);
        return true;
      }
      return false;
    },
    hasAudioElements: () => isIOS ? (audioContext !== null) : (audioElements.length > 0),
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
        
        if (isIOS) {
          // Nothing special needed for WebAudio - sources will complete naturally
          isPlaying = false;
        } else {
          // Stop any current playback for HTML5 Audio
          audioElements.forEach(audio => {
            if (!audio.paused) {
              try {
                audio.pause();
                if (audio.src && audio.src.startsWith('blob:')) {
                  URL.revokeObjectURL(audio.src);
                  audio.src = '';
                }
              } catch (e) {
                // Ignore errors during pause
              }
              audio.dataset.available = 'true';
            }
          });
          isPlaying = false;
        }
      }
      
      return true;
    },
    // Add explicit method to stop all audio playback immediately
    stopAllAudio: stopAllAudio,
    
    // Force unlock audio context for iOS - improved version for external calls
    unlockAudio: async () => {
      if (isIOS) {
        // Try multiple times with increasing delays if needed
        for (let attempts = 0; attempts < 3; attempts++) {
          const success = await unlockIOSAudio();
          if (success) return true;
          await new Promise(resolve => setTimeout(resolve, 300 * (attempts + 1)));
        }
        return false;
      }
      return true;
    },
    
    // Add method to properly reinitialize audio after stopping
    reinitializeAudio: reinitializeAudio,
    
    // Add method to force playback of silent sound to keep audio system active
    keepAudioActive: () => {
      if (isIOS && audioContext) {
        try {
          const silentBuffer = audioContext.createBuffer(1, 1, audioContext.sampleRate);
          const silentSource = audioContext.createBufferSource();
          silentSource.buffer = silentBuffer;
          silentSource.connect(audioContext.destination);
          silentSource.start(0);
          return true;
        } catch (e) {
          console.warn('Error keeping audio active:', e);
          return false;
        }
      }
      return true;
    }
  };
})();