(function() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const host = window.location.hostname;
    const port = 4000; // Same as in audioStreamBridge.js
    const ws = new WebSocket(`${protocol}//${host}:${port}`);
  
    ws.binaryType = 'arraybuffer';
  
    ws.onmessage = function(event) {
      // Received ArrayBuffer, play as audio
      const audioData = event.data;
      const blob = new Blob([audioData], { type: 'audio/wav' });
      const url = URL.createObjectURL(blob);
  
      const audio = new Audio(url);
      audio.play();
      audio.onended = () => URL.revokeObjectURL(url);
    };
  
    ws.onopen = () => console.log('Connected to audio stream');
    ws.onclose = () => console.log('Audio stream disconnected');
  })();