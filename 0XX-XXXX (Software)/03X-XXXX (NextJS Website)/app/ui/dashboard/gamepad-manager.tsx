"use client";

import { useEffect } from "react";

export default function GamepadManager({
    buttonL,
    buttonR,
    arrowLine1,
    arrowCircle1,
    arrowLine2,
    arrowCircle2,
}: any) {
    useEffect(() => {
        gamepadLoop(
            buttonL,
            buttonR,
            arrowLine1,
            arrowCircle1,
            arrowLine2,
            arrowCircle2
        );
    }, []);
    return <></>;
}
