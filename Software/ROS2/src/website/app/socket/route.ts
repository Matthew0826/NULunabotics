// Web socket server code

import { IncomingMessage } from "node:http";
import WebSocket, { WebSocketServer } from "ws";
import { lidarPoints, publishMockObstacle, publishToROS2, sendPathfindingRequest, sendPlanAction } from "../lib/ros2";
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
        // console.log("received: ", messageString);
        try {
            const messageJson = JSON.parse(messageString);
            if (messageJson.type === "sendPathfindingRequest") {
                const point1 = messageJson.message.point1;
                const point2 = messageJson.message.point2;
                const robot = messageJson.message.robot;
                const isPoint2BelowObstacles = point2[1] >= 244.0;
                const isPoint2DumpZone = point2[0] >= 274.0;
                if (isPoint2BelowObstacles) {
                    if (isPoint2DumpZone) {
                        console.log("Sending plan to dump zone");
                        sendPlanAction({ x: robot.x, y: robot.y }, false, true);
                    } else {
                        console.log("Sending plan to excavation zone");
                        sendPlanAction({ x: robot.x, y: robot.y }, true, false);
                    }
                } else {
                    // console.log("Sending pathfinding request");
                    // sendPathfindingRequest(
                    //     point1,
                    //     point2,
                    //     (points) => {
                    //         sendToClient("path", points);
                    //     }
                    // )
                    console.log("Making mock obstacle");
                    publishMockObstacle({ x: point2[0], y: point2[1], radius: 20 });
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
