import { IncomingMessage } from "node:http";
import WebSocket, { WebSocketServer } from "ws";
// import { lidarPoints } from "../lib/ros2";

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
        try {
            const messageJson = JSON.parse(messageString);
            console.log("in " + (Date.now() - messageJson?.timestamp) + "ms");
        } catch (error) {}
        // publishToROS2(messageString);
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
                // {
                //     graph: "Lidar",
                //     dataSet: "Points",
                //     newData: lidarPoints,
                // },
            ])
        );
        counter++;
    }, 1000);

    return () => clearInterval(interval);
}
