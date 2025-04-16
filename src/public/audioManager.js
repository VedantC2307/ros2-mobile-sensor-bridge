class AudioManager {
    constructor() {
        this.audioStarted = false;
        this.recognizer = null;
        this.keyword = "robot";
        this.finalTranscripts = "";
        this.transcriptionTimer = null;
        this.logDiv = document.getElementById('transcription-log');
        console.log('Audio Manager initialized');
    }

    logTranscription(prompt) {
        // Log to console
        console.log(`Transcribed: "${prompt}"`);
        console.log(`Timestamp: ${new Date().toISOString()}`);
        
        // Log to UI
        const timestamp = new Date().toLocaleTimeString();
        const logEntry = document.createElement('div');
        logEntry.className = 'log-entrytimestamp';
        logEntry.innerHTML = `<span class="timestamp">${timestamp}</span>${prompt}`;
        
        this.logDiv.insertBefore(logEntry, this.logDiv.firstChild);
        
        // Keep only last 10 entries
        while (this.logDiv.children.length > 10) {
            this.logDiv.removeChild(this.logDiv.lastChild);
        }
    }

    async startAudio(ws, isSessionActive) {
        if (this.audioStarted || !isSessionActive) return;
        
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
                                const timestamp = Date.now(); // Declare and initialize the timestamp variable
                                ws.send(JSON.stringify({ 
                                    header: {
                                        stamp: {
                                            sec: Math.floor(now / 1000),
                                            nanosec: (now % 1000) * 1000000
                                        },
                                        frame_id: 'audio_frame'
                                    },
                                    transcription: prompt 
                                }));
                                console.log('Sent transcribed prompt:', prompt);
                                this.logTranscription(prompt);
                            }
                            this.finalTranscripts = "";
                        }, 2000);timestamp
                    }
                }
            };

            this.recognizer.onerror = (event) => {
                console.error('Recognition error:', event.error);
                this.logTranscription(`Error: ${event.error}`);
            };

            this.recognizer.onend = () => {
                // Auto-restart if still active
                if (this.audioStarted) {
                    setTimeout(() => this.recognizer.start(), 10);
                }
            };

            this.recognizer.start();
            this.audioStarted = true;
            console.log('Speech recognition started');
            
        } catch (err) {
            console.error('Speech recognition error:', err);
            this.logTranscription(`Failed to start: ${err.message}`);
            alert('Speech recognition failed: ' + err.message);
        }
    }

    stopAudio() {
        if (this.recognizer) {
            this.audioStarted = false;
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

window.AudioManager = AudioManager;
timestamp