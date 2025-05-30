// Web socket client code

"use client";

import {
    createContext, ReactNode,
    useContext,
    useEffect,
    useRef,
    useState,
} from "react";
import { BatteryROSMessage, ObstacleROSMessage, ROSSocketMessage } from "@/app/types/sockets";
import { tempStartingData } from "@/app/lib/temp-graph-info";

type WebSocketContextType = {
    // allMessages: ROSSocketMessage[];
    obstacleMessages: ObstacleROSMessage[];
    batteryMessages: BatteryROSMessage[];
    latestMessages: ROSSocketMessage[];
    sendToServer: (messageType: string, message: any) => void;
};

const WebSocketContext = createContext<WebSocketContextType>({
    // allMessages: [],
    obstacleMessages: [],
    batteryMessages: [],
    latestMessages: [],
    sendToServer: () => { },
});

export const useWebSocketContext = () => useContext(WebSocketContext);

// This is like the client of the web socket
// It lets the user send and receive messages to/from the server
export default function WebSocketProvider({
    children,
}: {
    children: ReactNode;
}) {
    const socketRef = useRef<WebSocket | null>(null);
    useEffect(() => {
        if (typeof window === "undefined") return;

        const url = `ws://${window.location.host}/socket`;
        console.log(url);
        socketRef.current = new WebSocket(url);
        return () => {
            socketRef.current?.close();
            socketRef.current = null;
        };
    }, []);

    // const [messages, setMessages] = useState<ROSSocketMessage[]>(
    //     tempStartingData
    // );
    const [batteryMessages, setBatteryMessages] = useState<BatteryROSMessage[]>([]);
    const [obstacleMessages, setObstacleMessages] = useState<ObstacleROSMessage[]>([]);
    const [latestMessages, setLatestMessages] = useState<ROSSocketMessage[]>([]);

    const messageBuffer = useRef<any[]>([]);
    const flushing = useRef(false);

    useEffect(() => {
        async function handleMessage(event: MessageEvent) {
            const payload =
                typeof event.data === "string"
                    ? event.data
                    : await event.data.text();
            const messageList = JSON.parse(payload) as (ROSSocketMessage[] | undefined);

            if (!messageList) {
                console.error("Received undefined message list");
                return;
            }

            let latestMessagesThisBatch: ROSSocketMessage[] = [];
            messageList.forEach((message: ROSSocketMessage) => {
                if (message.type === "obstacles") {
                    setObstacleMessages((prev) => [...prev, message].splice(-500));
                } else if (message.type === "battery") {
                    setBatteryMessages((prev) => [...prev, message].splice(-10));
                } else if (message.type === "resetAutonomous") {
                    latestMessagesThisBatch.push(message);
                } else {
                    // setMessages((prev) => [...(prev.filter(a => a.type != message.type)), message]);
                    latestMessagesThisBatch = [...latestMessagesThisBatch.filter(a => a.type != message.type), message];
                }
            });

            if (latestMessagesThisBatch.length > 0) {
                setLatestMessages(latestMessagesThisBatch);
            }
        }

        socketRef.current?.addEventListener("message", handleMessage);
        socketRef.current?.addEventListener("open", () => {
            console.log("WebSocket connection opened");
        });
        return () =>
            socketRef.current?.removeEventListener("message", handleMessage);
    }, []);

    // setInterval(() => {
    //     if (messageBuffer.current.length > 0 && !flushing.current) {
    //         flushing.current = true;
    //         setMessages((prev) => [
    //             ...prev,
    //             ...messageBuffer.current.splice(0),
    //         ]);
    //         console.log("Flushing messages", messageBuffer.current);
    //         flushing.current = false;
    //     }
    // }, 10);

    function sendToServer(messageType: string, message: any) {
        console.log("Sending message to server:", messageType, message);
        try {
            if (socketRef.current?.readyState !== WebSocket.OPEN) {
                console.log("WebSocket is not open. Unable to send message.");
                return;
            }
            if (messageType === "resetAutonomous") {
                setObstacleMessages([]);
                setBatteryMessages([]);
                setLatestMessages([]);
            }
            socketRef.current?.send(JSON.stringify({ type: messageType, message: message }));
        } catch (error) {
            console.error("Error sending message to server:", error);
        }
    }

    return (
        <WebSocketContext.Provider value={{
            // allMessages: messages,
            obstacleMessages,
            batteryMessages,
            latestMessages,
            sendToServer
        }}>
            {children}
        </WebSocketContext.Provider>
    );
}
