"use client";

import { useCallback, useEffect, useRef, useState } from "react";

export default function TesterMessages() {
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
    const onMessage = useCallback((message: string) => {
        socketRef.current?.send(message);
        setMessages((messages) => [...messages, message]);
    }, []);

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
            socketRef.current?.send("hello nerds");
        });
        return () =>
            socketRef.current?.removeEventListener("message", handleMessage);
    }, []);

    useEffect(() => {
        // send a message on key press
        function handleKeyDown(event: KeyboardEvent) {
            socketRef.current?.send(event.key);
            console.log("sending", event.key);
        }
        window.addEventListener("keydown", handleKeyDown);
        return () => window.removeEventListener("keydown", handleKeyDown);
    });

    return (
        <div style={{ maxWidth: "50vh" }}>
            {messages.map((message, index) => (
                <p key={index}>{message}</p>
            ))}
        </div>
    );
}
