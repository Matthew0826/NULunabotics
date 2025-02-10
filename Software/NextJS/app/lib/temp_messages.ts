// write to the web socket every 3 seconds
export function sendMessageEvery3Seconds(socket: WebSocket) {
    setInterval(() => {
        console.log("sent!");
        socket.send("Hello from the server!");
    }, 3000);
}
