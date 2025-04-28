class SpeechRecognitionManager {
    constructor() {
        this.speechRecognitionStarted = false;
        this.recognizer = null;
        this.keyword = "robot"; // Default value that will be updated from config
        this.finalTranscripts = "";
        this.transcriptionTimer = null;
        this.logDiv = document.getElementById('transcription-log');
        console.log('Speech Recognition Manager initialized');
        
        // Load the keyword from config file
        this.loadConfigFromServer();
    }

    async loadConfigFromServer() {
        try {
            const response = await fetch('/api/config');
            if (response.ok) {
                const config = await response.json();
                if (config.microphone && config.microphone.wake_word) {
                    this.keyword = config.microphone.wake_word;
                    console.log(`Wake word loaded from config: "${this.keyword}"`);
                }
            } else {
                console.error('Failed to load config:', response.statusText);
            }
        } catch (error) {
            console.error('Error loading config:', error);
        }
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

            this.recognizer.onstart = () => {
                console.log('Speech recognition started');
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
            };

            this.recognizer.onend = () => {
                // Auto-restart if still active
                if (this.speechRecognitionStarted) {
                    setTimeout(() => this.recognizer.start(), 10);
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
        // Clear log when stopping
        if (this.logDiv) {
            this.logDiv.innerHTML = '';
        }
        console.log('Speech recognition stopped');
        this.logTranscription('Speech recognition stopped');
    }
}

window.SpeechRecognitionManager = SpeechRecognitionManager;