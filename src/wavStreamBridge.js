const rclnodejs = require('rclnodejs');
const WebSocket = require('ws');

async function main() {
  await rclnodejs.init();
  const node = rclnodejs.createNode('audio_stream_bridge');
  const wss = new WebSocket.Server({ port: 4000 }); // Choose a port

  let clients = new Set();

  wss.on('connection', (ws) => {
    clients.add(ws);
    ws.on('close', () => clients.delete(ws));
  });

  node.createSubscription(
    'std_msgs/msg/UInt8MultiArray',
    'wav_bytes',
    (msg) => {
      // msg.data is an array of uint8
      const buffer = Buffer.from(msg.data);
      // Send as binary to all clients
      clients.forEach(ws => {
        if (ws.readyState === WebSocket.OPEN) {
          ws.send(buffer);
        }
      });
    }
  );

  rclnodejs.spin(node);
  console.log('Audio stream bridge running on ws://<server-ip>:4000');
}

main();