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

    useEffect(() => {
        async function handleMessage(event: MessageEvent) {
            const payload =
                typeof event.data === "string"
                    ? event.data
                    : await event.data.text();
            setMessages((p) => [...p, JSON.parse(payload)]);
        }

        socketRef.current?.addEventListener("message", handleMessage);
        socketRef.current?.addEventListener("open", () => {
            console.log("WebSocket connection opened");
        });
        return () =>
            socketRef.current?.removeEventListener("message", handleMessage);
    }, []);

    function sendToServer(messageType: string, message: any) {
        socketRef.current?.send(JSON.stringify({ type: messageType, message: message }));
    }

    return (
        <WebSocketContext.Provider value={{ messages, sendToServer }}>
            {children}
        </WebSocketContext.Provider>
    );
}
