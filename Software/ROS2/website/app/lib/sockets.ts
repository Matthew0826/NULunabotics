import WebSocket from "ws";

export const sockets = new Set<WebSocket>();

export function sendToClient(message: string) {
    sockets.forEach((client) => {
        if (client.readyState === WebSocket.OPEN) {
            client.send(message);
        }
    });
}
