// Web socket client code

"use client";

import {
    createContext,
    useContext,
    useEffect,
    useRef,
    useState,
} from "react";

export type WebSocketMessages = {
    messages: string[];
    sendToServer: (messageType: string, message: any) => void;
};

const WebSocketContext = createContext<WebSocketMessages>({
    messages: [],
    sendToServer: () => { },
});

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

    const [messages, setMessages] = useState<string[]>([
        JSON.stringify([
            {
                graph: "Lidar",
                dataSet: "Points",
                newData: [
                    { distance: 2000, angle: (7 * Math.PI) / 4, weight: 0 },
                    { distance: 1000, angle: (6 * Math.PI) / 4, weight: 0 },
                    { distance: 2000, angle: (5 * Math.PI) / 4, weight: 0 },
                    { distance: 1200, angle: Math.PI, weight: 0 },
                    { distance: 2000, angle: (3 * Math.PI) / 4, weight: 0 },
                    { distance: 2000, angle: (2 * Math.PI) / 4, weight: 0 },
                    { distance: 2000, angle: Math.PI / 4, weight: 0 },
                    { distance: 2000, angle: 0, weight: 0 },
                ],
            },
        ]),
    ]);

    useEffect(() => {
        async function handleMessage(event: MessageEvent) {
            const payload =
                typeof event.data === "string"
                    ? event.data
                    : await event.data.text();
            setMessages((p) => [...p, payload]/*.slice(-10)*/); // Keep only the last 10 messages
        }

        socketRef.current?.addEventListener("message", handleMessage);
        socketRef.current?.addEventListener("open", () => {
            console.log("WebSocket connection opened");
        });
        return () =>
            socketRef.current?.removeEventListener("message", handleMessage);
    }, []);

    function sendToServer(messageType: string, message: any) {
        socketRef.current?.send(JSON.stringify({ type: messageType, data: message }));
    }

    return (
        <WebSocketContext.Provider value={{ messages, sendToServer }}>
            {children}
        </WebSocketContext.Provider>
    );
}
