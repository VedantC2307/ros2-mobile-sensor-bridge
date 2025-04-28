class TextToSpeech {
    constructor() {
        this.synth = window.speechSynthesis;
        this.voices = [];
        this.ws = null;
        this.isReady = false;
        this.autoReconnect = true;
        this.isSpeaking = false;
        this.audioConfig = { mode: 'tts', enabled: true }; // Default, will be updated
        
        // Initialize when created
        this.init();
    }

    async init() {
        // Load config and initialize voices
        await this.fetchConfig();
        
        // Only initialize if we're in TTS mode
        if (this.audioConfig.mode === 'tts' && this.audioConfig.enabled) {
            await this.initVoices();
            console.log('TTS system initialized and ready');
        } else {
            console.log('TTS system disabled by configuration');
        }
    }

    async fetchConfig() {
        try {
            const response = await fetch('/api/config');
            const config = await response.json();
            this.audioConfig = config.audio || this.audioConfig;
            console.log('TTS loaded audio configuration:', this.audioConfig);
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
                    console.log("TTS voices loaded:", this.voices.length);
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

    speak(text) {
        // Check if TTS is enabled in config
        if (this.audioConfig.mode !== 'tts' || !this.audioConfig.enabled) {
            console.log("TTS disabled by configuration, skipping speech:", text);
            return;
        }
        
        if (!this.synth || !this.isReady) {
            console.error("Speech synthesis not available or not ready");
            return;
        }

        // Cancel any ongoing speech
        this.synth.cancel();
        
        console.log("Speaking text:", text);

        const utterance = new SpeechSynthesisUtterance(text);
        // Try to find an English voice, fallback to first available
        utterance.voice = this.voices.find(voice => voice.lang.startsWith('en')) || this.voices[0];
        utterance.rate = 1;
        utterance.pitch = 1;
        utterance.volume = 1;
        
        // Set up event handlers for speech
        this.isSpeaking = true;
        
        utterance.onend = () => {
            this.isSpeaking = false;
        };
        
        utterance.onerror = (e) => {
            console.error("Speech error:", e);
            this.isSpeaking = false;
            this.fallbackSpeak(text);
        };
        
        try {
            this.synth.speak(utterance);
        } catch (err) {
            console.error("Exception during speak call:", err);
            this.fallbackSpeak(text);
        }
    }
    
    // Fallback method to try if the main speak method fails
    fallbackSpeak(text) {
        console.log("Trying fallback speech method");
        try {
            // Try a simpler approach without all the custom settings
            const simple = new SpeechSynthesisUtterance(text);
            this.synth.cancel(); // Cancel any ongoing speech
            this.synth.speak(simple);
        } catch (err) {
            console.error("Fallback speech also failed:", err);
        }
    }

    async connectWebSocket() {
        // Check the latest config before connecting
        await this.fetchConfig();
        
        // Only connect if TTS is enabled in configuration
        if (this.audioConfig.mode !== 'tts' || !this.audioConfig.enabled) {
            console.log("TTS disabled by configuration, not connecting WebSocket");
            return;
        }
        
        // Wait for voices to be loaded before connecting
        if (!this.isReady) {
            console.log("Voices not ready, initializing before connecting WebSocket");
            await this.initVoices();
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
                
                // Speak a confirmation message
                this.speak("Text to speech system ready");
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
            try {
                this.ws.close();
            } catch (e) {
                console.error('Error closing WebSocket:', e);
            }
            this.ws = null;
            
            if (this.audioConfig.mode === 'tts' && this.audioConfig.enabled) {
                this.speak("TTS system disconnected"); // Add disconnect message
            }
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

// Automatically create TTS instance and make it available globally
window.tts = new TextToSpeech();
