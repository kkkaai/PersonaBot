// web_server/static/js/main.js

var socket = io();

function sendCommand(command) {
    socket.emit('command', command);
}
