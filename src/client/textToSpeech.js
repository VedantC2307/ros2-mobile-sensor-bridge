class TextToSpeech {
    constructor() {
        this.synth = window.speechSynthesis;
        this.voices = [];
        this.ws = null;
        this.isReady = false;
        this.autoReconnect = true;
        this.isSpeaking = false;
        this.audioConfig = { mode: 'tts', enabled: true }; // Default, will be updated
        this.audioUnlocked = false;
        this.hasSpokenReady = false; // Track if we've already spoken the ready message
        this.hasSpokenDisconnect = false; // Track disconnect message
        this.initInProgress = false; // Prevent multiple concurrent initializations
        this.selectedVoice = null; // Store selected voice for consistency
        
        // Track the last spoken messages with timestamps to prevent duplicates
        this.lastSpokenMessages = new Map();
        
        // Default TTS parameters
        this.ttsParams = {
            rate: 1.0,
            pitch: 1.0,
            volume: 1.0,
            voice_preference: ""
        };
        
        // Initialize when created - but not immediately on iOS to avoid premature audio setup
        if (!this.isIOSDevice()) {
            this.init();
        } else {
            console.log("iOS device detected, deferring full TTS initialization until user interaction");
            // Just load config but wait for user interaction to initialize voices
            this.fetchConfig();
        }
    }

    async init() {
        // Prevent multiple concurrent initializations
        if (this.initInProgress) {
            console.log("TTS initialization already in progress, skipping duplicate call");
            return;
        }
        this.initInProgress = true;
        
        try {
            // Load config and initialize voices
            await this.fetchConfig();
            
            // Only initialize if we're in TTS mode
            if (this.audioConfig.mode === 'tts' && this.audioConfig.enabled) {
                await this.initVoices();
                console.log('TTS system initialized and ready');
                
                // Setup iOS audio unlock handlers - once only
                this.setupAudioUnlocking();
            } else {
                console.log('TTS system disabled by configuration');
            }
        } catch (err) {
            console.error("Error during TTS initialization:", err);
        }
        
        this.initInProgress = false;
    }
    
    setupAudioUnlocking() {
        // iOS requires user interaction to enable audio - but only set up once
        if (this._unlockListenersSet) return;
        
        const unlockAudio = () => {
            if (this.audioUnlocked) return;
            
            console.log("User interaction detected, unlocking audio for iOS");
            
            // Create a silent audio context for iOS
            const AudioContext = window.AudioContext || window.webkitAudioContext;
            if (AudioContext) {
                const audioCtx = new AudioContext();
                // Create and play a silent sound to unlock audio
                const silentBuffer = audioCtx.createBuffer(1, 1, 22050);
                const source = audioCtx.createBufferSource();
                source.buffer = silentBuffer;
                source.connect(audioCtx.destination);
                source.start(0);
                console.log("Audio context unlocking attempted");
            }
            
            // Also unlock the speech synthesis but don't speak anything
            if (this.synth) {
                this.synth.cancel(); // Cancel any pending speech
                // We deliberately don't speak here - will only speak when requested
            }
            
            // Make sure voices are loaded
            if (!this.isReady) {
                this.init();
            }
            
            this.audioUnlocked = true;
            
            // Remove event listeners after unlock attempt to prevent multiple unlocks
            this.removeAudioUnlockListeners();
        };
        
        // Store handlers so we can remove them later
        this._unlockHandler = unlockAudio;
        
        // Add event listeners to unlock audio on first user interaction
        document.addEventListener('touchend', this._unlockHandler, false);
        document.addEventListener('click', this._unlockHandler, false);
        
        // Mark that listeners are set
        this._unlockListenersSet = true;
    }
    
    removeAudioUnlockListeners() {
        if (this._unlockHandler) {
            document.removeEventListener('touchend', this._unlockHandler);
            document.removeEventListener('click', this._unlockHandler);
            console.log("Audio unlock listeners removed");
            this._unlockHandler = null;
        }
    }

    async fetchConfig() {
        try {
            const response = await fetch('/api/config');
            const config = await response.json();
            this.audioConfig = config.audio || this.audioConfig;
            
            // Get TTS specific parameters if available
            if (config.audio && config.audio.tts) {
                this.ttsParams = {
                    rate: parseFloat(config.audio.tts.rate) || 1.0,
                    pitch: parseFloat(config.audio.tts.pitch) || 1.0,
                    volume: parseFloat(config.audio.tts.volume) || 1.0,
                    voice_preference: config.audio.tts.voice_preference || ""
                };
            }
            
            console.log('TTS loaded configuration:', {
                audioConfig: this.audioConfig,
                ttsParams: this.ttsParams
            });
            
            return this.audioConfig;
        } catch (error) {
            console.error('TTS error loading configuration:', error);
            return this.audioConfig;
        }
    }

    async initVoices() {
        return new Promise((resolve) => {
            const loadVoices = () => {
                this.voices = this.synth.getVoices();
                if (this.voices.length > 0) {
                    this.isReady = true;
                    // Log voice information to help with debugging
                    console.log(`TTS voices loaded: ${this.voices.length}`);
                    console.log('Available voices:', this.voices.map(v => `${v.name} (${v.lang})`));
                    
                    // Pre-select and store a consistent voice
                    // Try to find the preferred voice if specified
                    if (this.ttsParams.voice_preference && this.ttsParams.voice_preference.trim() !== "") {
                        console.log(`Looking for voice preference: "${this.ttsParams.voice_preference}"`);
                        
                        // Try exact match first
                        this.selectedVoice = this.voices.find(voice => 
                            voice.name === this.ttsParams.voice_preference
                        );
                        
                        // If not found, try partial match (case-insensitive)
                        if (!this.selectedVoice) {
                            this.selectedVoice = this.voices.find(voice => 
                                voice.name.toLowerCase().includes(this.ttsParams.voice_preference.toLowerCase()) || 
                                voice.voiceURI.toLowerCase().includes(this.ttsParams.voice_preference.toLowerCase())
                            );
                        }
                        
                        // If still not found, try common US English patterns
                        if (!this.selectedVoice && this.ttsParams.voice_preference.toLowerCase().includes('us english')) {
                            this.selectedVoice = this.voices.find(voice => 
                                voice.lang === 'en-US' && 
                                (voice.name.toLowerCase().includes('us') || 
                                 voice.name.toLowerCase().includes('english') ||
                                 voice.name.toLowerCase().includes('america'))
                            );
                        }
                        
                        if (this.selectedVoice) {
                            console.log(`Found preferred voice: ${this.selectedVoice.name} (${this.selectedVoice.lang})`);
                        } else {
                            console.warn(`Preferred voice "${this.ttsParams.voice_preference}" not found`);
                        }
                    }
                    
                    if (!this.selectedVoice) {
                        // No preference or preferred voice not found, use default English selection
                        console.log('Using fallback voice selection...');
                        this.selectedVoice = this.voices.find(voice => voice.lang === 'en-US') || 
                                           this.voices.find(voice => voice.lang.startsWith('en')) || 
                                           this.voices[0];
                    }
                    
                    console.log(`Selected voice: ${this.selectedVoice?.name || 'default'} (${this.selectedVoice?.lang || 'unknown'})`);
                    
                    if (this.isIOSDevice()) {
                        console.log("iOS voices available:");
                        this.voices.filter(v => v.lang.startsWith('en')).forEach((v, i) => 
                            console.log(`${i+1}. ${v.name} (${v.lang})`));
                    }
                    
                    resolve();
                } else {
                    console.warn("No voices available yet, will retry");
                    // Try again in a moment
                    setTimeout(loadVoices, 500);
                }
            };

            loadVoices(); // Try immediate loading
            
            // Set up event listener if voices aren't loaded yet
            if (this.voices.length === 0) {
                this.synth.onvoiceschanged = () => {
                    loadVoices();
                };
            }
        });
    }

    // Check if the same message was spoken recently (within the debounce time)
    isRecentlySpoken(text, debounceTime = 2000) {
        const now = Date.now();
        const lastSpoken = this.lastSpokenMessages.get(text);
        
        if (lastSpoken && (now - lastSpoken < debounceTime)) {
            console.log(`Preventing duplicate speech: "${text}" (spoken ${now - lastSpoken}ms ago)`);
            return true;
        }
        
        // Update the timestamp for this message
        this.lastSpokenMessages.set(text, now);
        
        // Clean up old messages (optional - prevents memory leak for many unique messages)
        for (const [msg, timestamp] of this.lastSpokenMessages.entries()) {
            if (now - timestamp > 10000) { // Remove messages older than 10 seconds
                this.lastSpokenMessages.delete(msg);
            }
        }
        
        return false;
    }

    speak(text, forceSpeak = false) {
        // Special handling for known system messages
        const isReadyMessage = text.includes("ready") && text.includes("system");
        const isDisconnectMessage = text.includes("disconnect");
        
        // Check for duplicates of system messages
        if (isReadyMessage && this.hasSpokenReady) {
            console.log("Skipping duplicate ready message");
            return;
        }
        
        if (isDisconnectMessage && this.hasSpokenDisconnect) {
            console.log("Skipping duplicate disconnect message");
            return;
        }
        
        // Additional debounce check for all messages to prevent duplicates
        if (this.isRecentlySpoken(text)) {
            return;
        }
        
        // Don't speak if:
        // 1. We're on iOS and audio isn't unlocked (unless forced)
        // 2. TTS is disabled in config
        // 3. Speech synthesis isn't ready
        if (this.isIOSDevice() && !this.audioUnlocked && !forceSpeak) {
            console.log("Audio not unlocked on iOS, can't speak yet:", text);
            return;
        }
        
        if (this.audioConfig.mode !== 'tts' || !this.audioConfig.enabled) {
            console.log("TTS disabled by configuration, skipping speech:", text);
            return;
        }
        
        if (!this.synth || !this.isReady) {
            console.error("Speech synthesis not available or not ready");
            // For iOS, try to initialize
            if (this.isIOSDevice()) {
                this.init();
            }
            return;
        }

        // Set flags for special messages
        if (isReadyMessage) {
            this.hasSpokenReady = true;
        }
        
        if (isDisconnectMessage) {
            this.hasSpokenDisconnect = true;
        }

        // Cancel any ongoing speech
        this.synth.cancel();
        
        console.log("Speaking text:", text);

        const utterance = new SpeechSynthesisUtterance(text);
        
        // Always use the consistently selected voice
        if (this.selectedVoice) {
            utterance.voice = this.selectedVoice;
        } else {
            // Fallback if no voice was selected during initialization
            utterance.voice = this.voices.find(voice => voice.lang.startsWith('en')) || this.voices[0];
            // Store this voice for future use
            this.selectedVoice = utterance.voice;
        }
        
        // Apply speech parameters from config
        utterance.rate = this.ttsParams.rate;
        utterance.pitch = this.ttsParams.pitch;
        utterance.volume = this.ttsParams.volume;
        
        // Log the voice and parameters being used
        console.log(`TTS using voice: ${utterance.voice?.name || 'default'}, rate: ${utterance.rate}, pitch: ${utterance.pitch}, volume: ${utterance.volume}`);
        
        // Set up event handlers for speech
        this.isSpeaking = true;
        
        utterance.onend = () => {
            this.isSpeaking = false;
            console.log(`Finished speaking: "${text}"`);
        };
        
        utterance.onerror = (e) => {
            console.error("Speech error:", e);
            this.isSpeaking = false;
            
            // Only try fallback if this isn't already a fallback attempt
            if (!utterance._isFallback) {
                this.fallbackSpeak(text);
            }
        };
        
        try {
            utterance._startTime = Date.now(); // Mark the start time for debugging
            this.synth.speak(utterance);
            
            // iOS Safari fix - sometimes speech doesn't start properly
            if (this.isIOSDevice()) {
                setTimeout(() => {
                    // If speech hasn't started after 250ms (or if we can detect it's not speaking)
                    if (!this.synth.speaking || (Date.now() - utterance._startTime > 250 && !this.isSpeechDetected)) {
                        console.log("iOS speech not started, trying fallback...");
                        this.fallbackSpeak(text);
                    }
                }, 250);
            }
        } catch (err) {
            console.error("Exception during speak call:", err);
            this.fallbackSpeak(text);
        }
    }
    
    // Fallback method to try if the main speak method fails
    fallbackSpeak(text) {
        console.log("Trying fallback speech method");
        
        // Check if this message was already spoken recently
        if (this.isRecentlySpoken(text)) {
            return;
        }
        
        try {
            // Try a simpler approach without all the custom settings
            const simple = new SpeechSynthesisUtterance(text);
            
            // Mark this as a fallback so we don't get into a recursive loop
            simple._isFallback = true;
            
            // Use the same voice for consistency
            if (this.selectedVoice) {
                simple.voice = this.selectedVoice;
            }
            
            // Still apply basic parameters
            simple.rate = this.ttsParams.rate;
            simple.volume = this.ttsParams.volume;
            
            this.synth.cancel(); // Cancel any ongoing speech
            this.synth.speak(simple);
        } catch (err) {
            console.error("Fallback speech also failed:", err);
        }
    }
    
    // Helper to detect iOS devices
    isIOSDevice() {
        return /iPad|iPhone|iPod/.test(navigator.userAgent) && !window.MSStream;
    }

    async unlockAudioForIOS() {
        if (this.isIOSDevice() && !this.audioUnlocked) {
            console.log('Manual unlock attempt for iOS audio');
            
            if (!this._unlockHandler) {
                this.setupAudioUnlocking();
            }
            
            // Try to unlock audio
            const AudioContext = window.AudioContext || window.webkitAudioContext;
            let unlockSuccessful = false;
            
            if (AudioContext) {
                try {
                    const audioCtx = new AudioContext();
                    const silentBuffer = audioCtx.createBuffer(1, 1, 22050);
                    const source = audioCtx.createBufferSource();
                    source.buffer = silentBuffer;
                    source.connect(audioCtx.destination);
                    source.start(0);
                    
                    // On iOS, check the AudioContext state
                    unlockSuccessful = audioCtx.state === 'running';
                    console.log("Audio context unlocking attempted, state:", audioCtx.state);
                } catch (err) {
                    console.error("Error during audio unlock:", err);
                    unlockSuccessful = false;
                }
            }
            
            // Also verify the synth is working
            try {
                if (this.synth) {
                    // Create and immediately cancel a test utterance to check permissions
                    const testUtterance = new SpeechSynthesisUtterance('');
                    this.synth.cancel(); // Clear any pending speech
                    this.synth.speak(testUtterance);
                    this.synth.cancel(); // Cancel the test utterance
                    
                    // If no errors were thrown, consider this part successful
                    unlockSuccessful = unlockSuccessful || true;
                }
            } catch (err) {
                console.error("Error testing speech synthesis:", err);
            }
            
            // Make sure voices are loaded
            if (!this.isReady) {
                await this.init();
            }
            
            // Only set audioUnlocked if we have some confirmation of success
            if (unlockSuccessful) {
                this.audioUnlocked = true;
                this.removeAudioUnlockListeners();
                console.log("iOS audio successfully unlocked");
            } else {
                console.log("iOS audio unlock attempt may not have been successful");
            }
            
            return unlockSuccessful;
        }
        return this.audioUnlocked;
    }

    async connectWebSocket() {
        // Check the latest config before connecting
        await this.fetchConfig();
        
        // Only connect if TTS is enabled in configuration
        if (this.audioConfig.mode !== 'tts' || !this.audioConfig.enabled) {
            console.log("TTS disabled by configuration, not connecting WebSocket");
            return;
        }
        
        // Don't reset the flags on every connection attempt - only when we know speech happened
        // This prevents the ready message from being spoken again if the connection drops
        
        // Wait for voices to be loaded before connecting
        if (!this.isReady) {
            console.log("Voices not ready, initializing before connecting WebSocket");
            await this.init();
        }
        
        this.autoReconnect = true;
        
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const host = window.location.host;
        const wsUrl = `${protocol}//${host}/tts`;
        
        console.log(`Connecting to WebSocket at ${wsUrl}`);
        
        try {
            this.ws = new WebSocket(wsUrl);
            
            this.ws.onopen = () => {
                console.log('TTS WebSocket connected');
                
                // Register with audio controller if available
                if (window.audioController) {
                    window.audioController.setTTS(this);
                }
                
                // Update UI status if available
                this.updateConnectionStatus('tts', 'connected');
                
                // Wait a moment before speaking to avoid race conditions
                setTimeout(() => {
                    // Only speak if the ready message hasn't been spoken yet
                    // Don't use forceSpeak here - let normal iOS checks happen
                    if (!this.hasSpokenReady) {
                        this.speak("Text to speech system ready");
                    }
                }, 500);
            };

            this.ws.onmessage = (event) => {
                console.log("Received WebSocket message:", event.data);
                try {
                    // Try to parse as JSON first
                    const jsonData = JSON.parse(event.data);
                    if (jsonData.text) {
                        this.speak(jsonData.text);
                    } else {
                        // If JSON but unknown format, speak it as text
                        this.speak(event.data);
                    }
                } catch (e) {
                    // Not JSON, treat as plain text
                    this.speak(event.data);
                }
            };

            this.ws.onerror = (error) => {
                console.error('TTS WebSocket error:', error);
                // Update UI status if available
                this.updateConnectionStatus('tts', 'disconnected');
            };
            
            this.ws.onclose = () => {
                console.log('TTS WebSocket closed');
                
                // Update UI status if available
                this.updateConnectionStatus('tts', 'disconnected');
                
                if (this.autoReconnect) {
                    console.log('Attempting to reconnect TTS WebSocket...');
                    setTimeout(() => this.connectWebSocket(), 1000);
                }
            };
            
            // Return a promise that resolves when connection is open
            return new Promise((resolve, reject) => {
                let handled = false;
                
                this.ws.addEventListener('open', () => {
                    if (!handled) {
                        handled = true;
                        resolve();
                    }
                });
                
                this.ws.addEventListener('error', (err) => {
                    if (!handled) {
                        handled = true;
                        reject(err);
                    }
                });
                
                // Add timeout to avoid hanging forever
                setTimeout(() => {
                    if (!handled) {
                        handled = true;
                        reject(new Error('TTS WebSocket connection timeout'));
                    }
                }, 5000);
            });
        } catch (err) {
            console.error('Exception during WebSocket creation:', err);
            this.updateConnectionStatus('tts', 'disconnected');
            throw err;
        }
    }

    disconnectWebSocket() {
        this.autoReconnect = false;  // Prevent reconnection attempts
        
        if (this.ws) {
            console.log('Disconnecting TTS WebSocket');
            
            // First, immediately cancel any ongoing speech
            if (this.synth) {
                try {
                    // Force cancel any ongoing speech before saying disconnect message
                    this.synth.cancel();
                    
                    // For iOS, we need more aggressive cancellation
                    if (this.isIOSDevice()) {
                        console.log("Using aggressive speech cancellation for iOS");
                        // Reset the speaking flag immediately
                        this.isSpeaking = false;
                        
                        // Create a new dummy utterance with empty text to reset the speech engine
                        // This helps when the audio context might be in a weird state after clicking outside
                        const resetUtterance = new SpeechSynthesisUtterance('');
                        resetUtterance.volume = 0; // Silent
                        resetUtterance.rate = 10; // Super fast
                        resetUtterance.onend = () => this.synth.cancel(); // Cancel again after it ends
                        this.synth.speak(resetUtterance);
                        this.synth.cancel(); // Cancel one more time for good measure
                    }
                } catch (e) {
                    console.error('Error cancelling speech:', e);
                }
            }
            
            // Only speak disconnect message if we weren't in the middle of speaking
            // and we haven't already spoken the disconnect message recently
            if (this.audioConfig.mode === 'tts' && this.audioConfig.enabled && 
                this.isReady && !this.hasSpokenDisconnect && !this.synth.speaking) {
                console.log("Speaking disconnect message");
                this.speak("TTS system disconnected");
                this.hasSpokenDisconnect = true;
            } else {
                console.log("Skipping disconnect message");
            }
            
            // Close the WebSocket connection immediately, don't wait for speech
            try {
                if (this.ws) {
                    this.ws.close();
                    this.ws = null;
                }
            } catch (e) {
                console.error('Error closing WebSocket:', e);
            }
            
            // Reset flags
            setTimeout(() => {
                if (this.hasSpokenDisconnect) {
                    this.hasSpokenDisconnect = false;
                }
            }, 1000);
        }
    }
    
    // Method to check if speech is in progress
    isCurrentlySpeaking() {
        return this.isSpeaking || this.synth.speaking;
    }
    
    // Use the global updateConnectionStatus function if available
    updateConnectionStatus(type, status) {
        if (window.updateConnectionStatus) {
            window.updateConnectionStatus(type, status);
        } else if (document.getElementById(`${type}-status`)) {
            const statusEl = document.getElementById(`${type}-status`);
            statusEl.className = `connection-status ${status}`;
            console.log(`${type.charAt(0).toUpperCase() + type.slice(1)} sensor ${status}`);
        } else {
            console.log(`Connection status: ${type} is ${status}`);
        }
    }
}

// Make updateConnectionStatus available globally
window.updateConnectionStatus = function(type, status) {
    const statusEl = document.getElementById(`${type}-status`);
    if (statusEl) {
        statusEl.className = `connection-status ${status}`;
    }
    
    // Log status changes to the console
    if (status === 'disconnected') {
        console.log(`${type.charAt(0).toUpperCase() + type.slice(1)} sensor disconnected`);
    } else if (status === 'connected') {
        console.log(`${type.charAt(0).toUpperCase() + type.slice(1)} sensor connected`);
    }
};

// Modified initialization: Only create the TTS instance, don't run init automatically
// Let user interaction trigger the full initialization on iOS
window.tts = new TextToSpeech();