const SerialPort = require('serialport')
const Readline = require('@serialport/parser-readline')
const express = require('express')
const http = require('http')
const WebSocket = require('ws')

const app = express();

app.use(require('body-parser').json());
app.use(require('express-static')('./'));
const server = http.createServer(app);

const wss = new WebSocket.Server({ server });

wss.on('connection', (ws) => {
    ws.on('message', (message) => {
            console.log('received: %s', message);
            port.write(message);
    });
    //send immediatly a feedback to the incoming connection    
    ws.send('Hi there, I am a WebSocket server');
});

server.listen(process.env.PORT || 8999, () => {
  console.log(`Server started :)`);
});

const port = new SerialPort('/dev/ttyUSB0', {baudRate: 9600})

port.on('error', function(err) {
	console.log('Error: ', err.message)
})

const parser = port.pipe(new Readline({ delimiter: '\r\n' }))
parser.on('data', function(data){
	wss.clients.forEach(client => {
		client.send(data);
	})
})


setInterval(function() {
	port.write('l', function(err) {
		if (err) {
			return console.log('Error on write: ', err.message)
		}
	})
}, 200);
