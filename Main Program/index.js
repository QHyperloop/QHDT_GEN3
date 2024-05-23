const express = require('express');
const { WebSocketServer } = require('ws');
const http = require('http');
const app = express();


app.use(express.static('public'));

app.get('/', (req, res) => {
    res.sendFile(__dirname + '/public/home-screen.html');
  });
  
app.get('/pod-interface', (req, res) => {
    res.sendFile(__dirname + '/public/pod-interface.html');
});

app.get('/error-log-warnings', (req, res) => {
    res.sendFile(__dirname + '/public/error-log-warnings.html');
});

app.get('/settings', (req, res) => {
    res.sendFile(__dirname + '/public/settings.html');
});


// Create an HTTP server from the Express app
const server = http.createServer(app);

// Attach WebSocket server to the HTTP server
const wss = new WebSocketServer({ server });

wss.on('connection', function connection(ws) {
    ws.on('message', function incoming(message) {
        console.log('received: %s', message);
        // Broadcast the message to all clients except the sender
        wss.clients.forEach(function each(client) {
            if (client !== ws && client.readyState === WebSocket.OPEN) {
                client.send(message);
            }
        });
    });

    // Send a welcome message to the newly connected client
    ws.send('WebSocket connection established');
});

// Listen on the same PORT for both HTTP and WebSocket
const PORT = process.env.PORT || 3000;
server.listen(PORT, () => {
  console.log(`Server is running on http://localhost:${PORT}`);
});