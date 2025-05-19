// Web socket client code

"use client";

import {
    createContext,
    useContext,
    useEffect,
    useRef,
    useState,
} from "react";
import { tempStartingData } from "./temp-graph-info";

export type WebSocketMessages = {
    messages: Message[];
    sendToServer: (messageType: string, message: any) => void;
};

const WebSocketContext = createContext<WebSocketMessages>({
    messages: [],
    sendToServer: () => { },
});

export type Message = {
    type: string;
    message: any;
};

export const useWebSocketContext = () => useContext(WebSocketContext);

// This is like the client of the web socket
// It lets the user send and receive messages to/from the server
export default function WebSocketProvider({
    children,
}: {
    children: React.ReactNode;
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

    const [messages, setMessages] = useState<Message[]>(
        tempStartingData
    );
    const messageBuffer = useRef<any[]>([]);
    const flushing = useRef(false);

    useEffect(() => {
        async function handleMessage(event: MessageEvent) {
            const payload =
                typeof event.data === "string"
                    ? event.data
                    : await event.data.text();
            const message = JSON.parse(payload);
            // messageBuffer.current.push(message);
            if (message.type === "obstacles") {
                setMessages((prev) => [...prev, message]);
            } else if (message.type === "battery") {
                setMessages((prev) => {
                    let powerMsgCount = 0;
                    const otherMessages = prev.filter((a) => a.type != message.type);
                    const powerMessages = [...prev.filter((a) => a.type == message.type), message].slice(-10);
                    return [...otherMessages, ...powerMessages];
                });
            } else if (message.type === "resetAutonomous") {
                setMessages([]);
            } else {
                setMessages((prev) => [...(prev.filter(a => a.type != message.type)), message]);
            }
        }

        socketRef.current?.addEventListener("message", handleMessage);
        socketRef.current?.addEventListener("open", () => {
            console.log("WebSocket connection opened");
        });
        return () =>
            socketRef.current?.removeEventListener("message", handleMessage);
    }, []);

    const interval = setInterval(() => {
        if (messageBuffer.current.length > 0 && !flushing.current) {
            flushing.current = true;
            setMessages((prev) => [
                ...prev,
                ...messageBuffer.current.splice(0),
            ]);
            flushing.current = false;
        }
    }, 20);

    function sendToServer(messageType: string, message: any) {
        try {
            if (socketRef.current?.readyState !== WebSocket.OPEN) {
                console.log("WebSocket is not open. Unable to send message.");
                return;
            }
            if (messageType === "resetAutonomous") {
                setMessages([]);
            }
            socketRef.current?.send(JSON.stringify({ type: messageType, message: message }));
        } catch (error) {
            console.error("Error sending message to server:", error);
        }
    }

    return (
        <WebSocketContext.Provider value={{ messages, sendToServer }}>
            {children}
        </WebSocketContext.Provider>
    );
}
