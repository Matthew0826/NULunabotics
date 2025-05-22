import WebSocket from "ws";

export const sockets = new Set<WebSocket>();

const messageBuffer = new Set<any>();

export function sendToClient(category: string, message: any, canBeLossy: boolean = false) {
    messageBuffer.add({
        type: category,
        message
    });
}

// make an interval that sends messages to the client every 33ms (33 frames per second)
export function startInterval() {
    setInterval(() => {
        sockets.forEach((client) => {
            if (client.readyState === WebSocket.OPEN && messageBuffer.size > 0) {
                // console.log("Sending message to client", messageBuffer);
                client.send(JSON.stringify(Array.from(messageBuffer)));
            }
        });
        messageBuffer.clear();
    }, 33);
}

startInterval();
