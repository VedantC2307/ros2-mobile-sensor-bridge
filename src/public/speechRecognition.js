class SpeechRecognitionManager {
    constructor() {
        this.speechRecognitionStarted = false;
        this.recognizer = null;
        this.keyword = "robot";
        this.finalTranscripts = "";
        this.transcriptionTimer = null;
        this.logDiv = document.getElementById('transcription-log');
        this.restartCount = 0;
        this.maxRestarts = 3; // Limit number of rapid restarts
        this.lastRestartTime = 0;
        this.restartDelay = 5000; // 5 seconds between restart attempts to reduce noise
        this.silentRestart = false; // Flag to control restart sound
        console.log('Speech Recognition Manager initialized');
    }

    logTranscription(prompt) {
        // Log to console
        console.log(`Transcribed: "${prompt}"`);
        console.log(`Timestamp: ${new Date().toISOString()}`);
        
        // Log to UI
        const timestamp = new Date().toLocaleTimeString();
        const logEntry = document.createElement('div');
        logEntry.className = 'log-entry';
        logEntry.innerHTML = `<span class="timestamp">${timestamp}</span>${prompt}`;
        
        this.logDiv.insertBefore(logEntry, this.logDiv.firstChild);
        
        // Keep only last 10 entries
        while (this.logDiv.children.length > 10) {
            this.logDiv.removeChild(this.logDiv.lastChild);
        }
    }

    async startSpeechRecognition(ws, isSessionActive) {
        if (this.speechRecognitionStarted || !isSessionActive) return;
        
        if (!("webkitSpeechRecognition" in window)) {
            throw new Error('Speech Recognition API not supported');
        }

        try {
            this.recognizer = new webkitSpeechRecognition();
            this.recognizer.continuous = true;
            this.recognizer.interimResults = false;
            this.recognizer.lang = "en-US";
            
            // Add property to control audio start feedback sound
            this.silentRestart = true;
            
            // Enable continuous without the notification sound
            if (SpeechRecognition && SpeechRecognition.prototype) {
                try {
                    // Some browsers may support this undocumented feature
                    this.recognizer.audiostart = () => {
                        console.log('Audio started');
                    };
                } catch (e) {
                    console.log('AudioStart event not supported');
                }
            }

            this.recognizer.onstart = () => {
                console.log('Speech recognition started');
                this.restartCount = 0; // Reset restart counter on successful start
            };

            this.recognizer.onresult = (event) => {
                for(let i = event.resultIndex; i < event.results.length; i++) {
                    const transcript = event.results[i][0].transcript;
                    
                    if(event.results[i].isFinal && 
                       (transcript.toLowerCase().includes(this.keyword) || 
                        this.finalTranscripts.toLowerCase().includes(this.keyword))) {
                        
                        console.log('Raw transcript:', transcript);
                        this.finalTranscripts += transcript;
                        
                        if (this.transcriptionTimer) {
                            clearTimeout(this.transcriptionTimer);
                        }
                        
                        this.transcriptionTimer = setTimeout(() => {
                            const split_arr = this.finalTranscripts.split(" ");
                            const key_idx = split_arr.findIndex(
                                (word) => word.toLowerCase() === this.keyword
                            );
                            const prompt_arr = split_arr.splice(key_idx + 1);
                            const prompt = prompt_arr.join(" ");

                            if (ws && ws.readyState === WebSocket.OPEN) {
                                const now = Date.now();
                                ws.send(JSON.stringify({ 
                                    header: {
                                        stamp: {
                                            sec: Math.floor(now / 1000),
                                            nanosec: (now % 1000) * 1000000
                                        },
                                        frame_id: 'microphone_frame'
                                    },
                                    transcription: prompt 
                                }));
                                console.log('Sent transcribed prompt:', prompt);
                                this.logTranscription(prompt);
                            }
                            this.finalTranscripts = "";
                        }, 2000);
                    }
                }
            };

            this.recognizer.onerror = (event) => {
                console.error('Recognition error:', event.error);
                this.logTranscription(`Error: ${event.error}`);
                
                // Don't restart on certain errors
                if (event.error === 'aborted' || event.error === 'not-allowed') {
                    this.speechRecognitionStarted = false;
                }
            };

            this.recognizer.onend = () => {
                // Control auto-restart with delay to avoid constant "kudung" sounds
                if (this.speechRecognitionStarted) {
                    const now = Date.now();
                    this.restartCount++;
                    
                    // If restarting too frequently, add increasing delays
                    if (now - this.lastRestartTime < 1000) {
                        // Restarting too quickly
                        if (this.restartCount > this.maxRestarts) {
                            console.log(`Too many rapid restarts (${this.restartCount}), adding delay`);
                            setTimeout(() => {
                                if (this.speechRecognitionStarted) {
                                    console.log('Restarting after delay period');
                                    this.recognizer.start();
                                    this.lastRestartTime = Date.now();
                                }
                            }, this.restartDelay);
                            return;
                        }
                    } else {
                        // Reset counter if it's been a while since last restart
                        this.restartCount = 0;
                    }
                    
                    console.log('Speech recognition ended, restarting...');
                    this.lastRestartTime = now;
                    
                    // Use a small delay to avoid continuous restart sounds
                    setTimeout(() => {
                        if (this.speechRecognitionStarted) {
                            try {
                                this.recognizer.start();
                            } catch (e) {
                                console.error('Error restarting speech recognition:', e);
                            }
                        }
                    }, 500);
                }
            };

            this.recognizer.start();
            this.speechRecognitionStarted = true;
            console.log('Speech recognition started');
            
        } catch (err) {
            console.error('Speech recognition error:', err);
            this.logTranscription(`Failed to start: ${err.message}`);
            alert('Speech recognition failed: ' + err.message);
        }
    }

    stopSpeechRecognition() {
        if (this.recognizer) {
            this.speechRecognitionStarted = false;
            this.recognizer.stop();
            this.recognizer = null;
        }
        if (this.transcriptionTimer) {
            clearTimeout(this.transcriptionTimer);
            this.transcriptionTimer = null;
        }
        this.finalTranscripts = "";
        this.restartCount = 0;
        // Clear log when stopping
        if (this.logDiv) {
            this.logDiv.innerHTML = '';
        }
        console.log('Speech recognition stopped');
        this.logTranscription('Speech recognition stopped');
    }
}

window.SpeechRecognitionManager = SpeechRecognitionManager;