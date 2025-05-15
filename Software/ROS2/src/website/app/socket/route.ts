// Web socket server code

import { IncomingMessage } from "node:http";
import WebSocket, { WebSocketServer } from "ws";
import { lidarPoints, publishMockObstacle, publishSimReset, publishToROS2, sendPathfindingRequest, sendPlanAction, startLoopingAction, stopLoopingAction } from "../lib/ros2";
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
                sendPlanAction({ x: robot.x, y: robot.y }, false, false, { x: point2[0], y: point2[1] });
                // console.log("Making mock obstacle");
                // publishMockObstacle({ x: point2[0], y: point2[1], radius: 20 });
            } else if (messageJson.type === "beginAutonomous") {
                startLoopingAction(true);
            } else if (messageJson.type === "stopAutonomous") {
                stopLoopingAction();
            } else if (messageJson.type === "resetAutonomous") {
                stopLoopingAction();
                publishSimReset(messageJson.message);
            } else if (messageJson.type === "controls") {
                publishToROS2(messageJson.message);
            }
        } catch (error) { }
    });

    console.log("Client connected");
    sockets.add(client);

    return () => { }; //clearInterval(interval);
}
