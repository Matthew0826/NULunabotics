import { IncomingMessage } from "node:http";
import WebSocket, { WebSocketServer } from "ws";
import { publishToROS2 } from "../lib/ros2";

export function GET() {
    const headers = new Headers();
    headers.set("Connection", "Upgrade");
    headers.set("Upgrade", "websocket");
    return new Response("Upgrade Required", { status: 426, headers });
}

// This is like the server of the web socket
// It can talk to all the clients and receive messages from them
export function SOCKET(
    client: WebSocket,
    _request: IncomingMessage,
    server: WebSocketServer
) {
    client.on("message", (message: WebSocket.RawData) => {
        const messageString = message.toString();
        console.log("received: ", messageString);
        publishToROS2(messageString);
    });

    let counter = 0;

    const interval = setInterval(() => {
        client.send(
            JSON.stringify([
                {
                    graph: "Power",
                    dataSet: "Test Data",
                    newData: [Math.floor(Math.random() * 30)],
                },
                {
                    graph: "Other Graph",
                    dataSet: "Sin",
                    newData: [Math.sin(counter / Math.PI) * 25],
                },
                {
                    graph: "Other Graph",
                    dataSet: "Cos",
                    newData: [Math.cos(counter / Math.PI) * 25],
                },
            ])
        );
        counter++;
    }, 1000);

    return () => clearInterval(interval);
}
