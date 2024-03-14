// Libraries
const express = require('express');
var can = require('socketcan');

// Express Calls
const back = express();
const port = 3000;

// Author: Ryan Silverberg, Kyle ______

/**
 * TODO:
 * Express
 * SocketCAN
 */

var channel = can.createRawChannel("vcan0", true); //Create CAN channel
channel.addListener("onMessage", function(msg) {console.log(msg)}); //Log any message
channel.addListener("onMessage", channel.send, channel); //Reply any message

back.get('/', (req, res) => {
  res.send('Hello World!');
})

back.listen(port, () => {
  console.log(`Example App listening on port ${port}`);
})


