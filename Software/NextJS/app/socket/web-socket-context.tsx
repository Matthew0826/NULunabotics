"use client";

import {
    createContext,
    useCallback,
    useContext,
    useEffect,
    useRef,
    useState,
} from "react";

export type WebSocketMessages = {
    messages: string[];
    sendToServer: (message: string) => void;
};

const WebSocketContext = createContext<WebSocketMessages>({
    messages: [],
    sendToServer: () => {},
});

export const useWebSocketContext = () => useContext(WebSocketContext);

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

    const [messages, setMessages] = useState<string[]>([]);

    useEffect(() => {
        async function handleMessage(event: MessageEvent) {
            const payload =
                typeof event.data === "string"
                    ? event.data
                    : await event.data.text();
            setMessages((p) => [...p, payload]);
        }

        socketRef.current?.addEventListener("message", handleMessage);
        socketRef.current?.addEventListener("open", () => {
            socketRef.current?.send("A new page opened to control the robot.");
        });
        return () =>
            socketRef.current?.removeEventListener("message", handleMessage);
    }, []);

    function sendToServer(message: string) {
        socketRef.current?.send(message);
    }

    return (
        <WebSocketContext.Provider value={{ messages, sendToServer }}>
            {children}
        </WebSocketContext.Provider>
    );
}
