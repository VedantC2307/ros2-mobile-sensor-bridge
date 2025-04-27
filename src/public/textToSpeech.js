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
                    console.log("Voices loaded:", this.voices.length);
                    resolve();
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
        
        utterance.onerror = () => {
            this.isSpeaking = false;
        };
        
        this.synth.speak(utterance);
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
            await this.initVoices();
        }
        
        this.autoReconnect = true;
        
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const host = window.location.host;
        this.ws = new WebSocket(`${protocol}//${host}/tts`);

        this.ws.onopen = () => {
            console.log('TTS WebSocket connected');
            // Register with audio controller if available
            if (window.audioController) {
                window.audioController.setTTS(this);
            }
            
            // Update UI status if available
            if (window.updateConnectionStatus) {
                window.updateConnectionStatus('tts', 'connected');
            }
            
            this.speak("TTS system ready"); // Confirmation message
        };

        this.ws.onmessage = (event) => {
            console.log("Received text to speak:", event.data);
            this.speak(event.data);
        };

        this.ws.onerror = (error) => {
            console.error('TTS WebSocket error:', error);
            // Update UI status if available
            if (window.updateConnectionStatus) {
                window.updateConnectionStatus('tts', 'disconnected');
            }
        };
        
        this.ws.onclose = () => {
            console.log('TTS WebSocket closed');
            
            // Update UI status if available
            if (window.updateConnectionStatus) {
                window.updateConnectionStatus('tts', 'disconnected');
            }
            
            if (this.autoReconnect) {
                console.log('Attempting to reconnect...');
                setTimeout(() => this.connectWebSocket(), 1000);
            }
        };
    }

    disconnectWebSocket() {
        this.autoReconnect = false;  // Prevent reconnection attempts
        if (this.ws) {
            this.ws.close();
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
}

// Automatically create TTS instance and make it available globally
window.tts = new TextToSpeech();
