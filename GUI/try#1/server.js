// server.js
const express = require('express');
const bodyParser = require('body-parser');

const app = express();
const port = 3000;

// Use body-parser middleware to parse JSON
app.use(bodyParser.json());

// In-memory storage for sensor data
const sensorData = {};

// Define an endpoint to receive sensor data
app.post('/sensor-data', (req, res) => {
    const data = req.body;

    // Validate the received data
    if (!data || !data.sensorType || !data.sensorValue) {
        return res.status(400).send('Invalid sensor data');
    }

    // Store the sensor data based on sensor type
    const sensorType = data.sensorType;
    if (!sensorData[sensorType]) {
        sensorData[sensorType] = [];
    }

    sensorData[sensorType].push(data.sensorValue);

    console.log(`Received sensor data [${sensorType}]:`, data);
    res.send('Data received successfully');
});

// Define an endpoint to get all sensor data
app.get('/sensor-data', (req, res) => {
    res.json(sensorData);
});

// Start the server
app.listen(port, () => {
    console.log(`Server is running at http://localhost:${port}`);
});
