import WebSocket from "ws";

export const sockets = new Set<WebSocket>();

export function sendToClient(category: string, message: any) {
    sockets.forEach((client) => {
        if (client.readyState === WebSocket.OPEN) {
            client.send(JSON.stringify({ type: category, message }));
        }
    });
}
