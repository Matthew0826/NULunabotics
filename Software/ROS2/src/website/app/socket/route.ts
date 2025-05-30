// Web socket server code

import { IncomingMessage } from "node:http";
import WebSocket, { WebSocketServer } from "ws";
import { publishCodeUpload, publishConfig, publishMockObstacle, publishSimReset, publishMotorsToROS2, publishOrientationCorrection, sendPathfindingRequest, sendPlanAction, startLoopingAction, stopLoopingAction } from "../lib/ros2";
import { sendToClient, sockets } from "../lib/sockets";
import { getConfigFromProfile, setConfigFromProfile } from "../lib/config-manager";
import { ROSSocketMessage } from "@/app/types/sockets";

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
            const messageJson = JSON.parse(messageString) as ROSSocketMessage;
            if (messageJson.type === "sendPathfindingRequest") {
                const point1 = messageJson.message.point1;
                const point2 = messageJson.message.point2;
                const robot = messageJson.message.robot;
                sendPlanAction({ x: robot.x, y: robot.y }, false, false, { x: point2[0], y: point2[1] });
            } else if (messageJson.type == "mockObstacle") {
                const point = messageJson.message.point;
                publishMockObstacle({ x: point[0], y: point[1], radius: 20 });
            } else if (messageJson.type === "beginAutonomous") {
                startLoopingAction(true);
            } else if (messageJson.type === "stopAutonomous") {
                stopLoopingAction();
            } else if (messageJson.type === "resetAutonomous") {
                stopLoopingAction();
                publishSimReset(messageJson.message);
            } else if (messageJson.type === "controls") {
                publishMotorsToROS2(messageJson.message);
            } else if (messageJson.type === "uploadCode") {
                publishCodeUpload(messageJson.message);
            } else if (messageJson.type === "orientationCorrection") {
                publishOrientationCorrection(messageJson.message);
            } else if (messageJson.type == "loadConfigProfile") {
                console.log(messageJson.message.profile);
                // based on the name of profile, returns information for that profile
                console.log("HERE YOU GO", messageJson.message.profile);
                sendToClient({
                    type: "loadConfigProfile",
                    message: {
                        profile: messageJson.message.profile,
                        config: getConfigFromProfile(messageJson.message.profile)
                    }
                });
            } else if (messageJson.type === "saveConfigProfile") {
                console.log("OK SERVER SAVE");
                setConfigFromProfile(messageJson.message.config, messageJson.message.profile);
            }
        } catch (error) {
            console.error("Error parsing message:", error);
        }
    });

    console.log("Client connected");
    sockets.add(client);

    // assume default
    sendToClient({
        type: "loadConfigProfile",
        message: {
            profile: "default",
            config: getConfigFromProfile("default")
        }
    });

    return () => {
    }; //clearInterval(interval);
}
