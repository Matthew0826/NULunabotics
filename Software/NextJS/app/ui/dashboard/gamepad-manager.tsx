"use client";

import { gamepadLoop } from "@/app/lib/utils";
import { useEffect, useRef } from "react";
import WebSocket from "ws";

export default function GamepadManager({
    buttonL,
    buttonR,
    arrowLine1,
    arrowCircle1,
    arrowLine2,
    arrowCircle2,
}: any) {
    const socketRef = useRef<WebSocket | null>(null);
    useEffect(() => {
        gamepadLoop(
            buttonL,
            buttonR,
            arrowLine1,
            arrowCircle1,
            arrowLine2,
            arrowCircle2,
            socketRef
        );
    }, []);
    return <></>;
}
