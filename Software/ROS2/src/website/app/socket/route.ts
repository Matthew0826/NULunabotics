// Web socket server code

import { IncomingMessage } from "node:http";
import WebSocket, { WebSocketServer } from "ws";
import { lidarPoints, publishToROS2, sendPathfindingRequest } from "../lib/ros2";
import { sockets } from "../lib/sockets";

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
    request: IncomingMessage,
    server: WebSocketServer,
    context: { params: Record<string, string | string[]> }
) {
    client.on("close", () => {
        console.log("Client disconnected");
        sockets.delete(client);
    });
    client.on("message", (message: WebSocket.RawData) => {
        const messageString = message.toString();
        console.log("received: ", messageString);
        try {
            const messageJson = JSON.parse(messageString);
            if (messageJson.type === "sendPathfindingRequest") {
                sendPathfindingRequest(
                    messageJson.data.point1,
                    messageJson.data.point2,
                    (points) => {
                        const response = JSON.stringify([
                            {
                                graph: "Path",
                                dataSet: "Pathfinding",
                                newData: points,
                            },
                        ]);
                        client.send(response);
                    }
                )
            } else if (messageJson.type === "controls") {
                publishToROS2(JSON.stringify(messageJson.data));
            }
        } catch (error) {}
    });

    console.log("Client connected");
    sockets.add(client);
    // const interval = setInterval(() => {
    //     client.send(
    //         JSON.stringify([
    //             {
    //                 graph: "Power",
    //                 dataSet: "Test Data",
    //                 newData: [Math.floor(Math.random() * 30)],
    //             }
    //         ])
    //     );
    // }, 1000);

    return () => {}; //clearInterval(interval);
}
