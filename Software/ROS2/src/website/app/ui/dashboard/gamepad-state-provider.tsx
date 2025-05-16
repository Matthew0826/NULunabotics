"use client";

import { useKeyboardController } from "@/app/lib/keyboard-controller";
import { gamepadLoop, normalizedVectorToPixels } from "@/app/lib/utils";
import { useWebSocketContext } from "@/app/lib/web-socket-context";
import { useContext, useState } from "react";
import { createContext } from "react";
import { useEffect, useRef } from "react";
import WebSocket from "ws";

export type GamepadState = {
    x1: number;
    y1: number;
    x2: number;
    y2: number;
    buttonL: boolean;
    buttonR: boolean;
    timestamp: number;
};

type GamepadManagerContextType = {
    state: GamepadState;
    setState: React.Dispatch<React.SetStateAction<GamepadState>>;
    speed: number;
    setSpeed: React.Dispatch<React.SetStateAction<number>>;
};

export const defaults = {
    x1: 100,
    y1: 100,
    x2: 100,
    y2: 100,
    buttonL: false,
    buttonR: false,
    timestamp: 0,
};


const GamepadManagerContext = createContext<GamepadManagerContextType>({
    state: defaults,
    setState: () => { },
    speed: 0,
    setSpeed: () => { },
});

export const useGamepadManagerContext = () => useContext(GamepadManagerContext);

export default function GamepadStateProvider({
    children,
}: {
    children: React.ReactNode;
}) {
    const { messages, sendToServer } = useWebSocketContext();
    const [state, setState] = useState<GamepadState>({
        x1: 0,
        y1: 0,
        x2: 0,
        y2: 0,
        buttonL: false,
        buttonR: false,
        timestamp: 0,
    });
    const [speed, setSpeed] = useKeyboardController();

    useEffect(() => {
        gamepadLoop(sendToServer, setState, speed);
    }, [speed]);
    return (
        <GamepadManagerContext.Provider value={{ state, setState, speed, setSpeed }}>
            {children}
        </GamepadManagerContext.Provider>
    );
}
