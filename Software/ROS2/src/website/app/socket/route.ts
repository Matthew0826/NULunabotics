// Web socket server code

import { IncomingMessage } from "node:http";
import WebSocket, { WebSocketServer } from "ws";
import { lidarPoints, publishToROS2, sendPathfindingRequest, sendPlanAction } from "../lib/ros2";
import { sendToClient, sockets } from "../lib/sockets";

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
                const isPoint2BelowObstacles = messageJson.message.point2.y >= 244.0;
                const isPoint2DumpZone = messageJson.message.point2.x >= 274.0;
                if (isPoint2BelowObstacles) {
                    if (isPoint2DumpZone) {
                        sendPlanAction(messageJson.message.point1, false, true);
                    } else {
                        sendPlanAction(messageJson.message.point1, true, false);
                    }
                } else {
                    sendPathfindingRequest(
                        messageJson.message.point1,
                        messageJson.message.point2,
                        (points) => {
                            sendToClient("path", points);
                        }
                    )
                }

            } else if (messageJson.type === "controls") {
                publishToROS2(messageJson.message);
            }
        } catch (error) { }
    });

    console.log("Client connected");
    sockets.add(client);

    return () => { }; //clearInterval(interval);
}
