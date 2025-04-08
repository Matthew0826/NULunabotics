"use client";

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
};

const defaults = {
    x1: 100,
    y1: 100,
    x2: 100,
    y2: 100,
    buttonL: false,
    buttonR: false,
    timestamp: 0,
};
const defaultsNormalized = {
    x1: 0,
    y1: 0,
    x2: 0,
    y2: 0,
    buttonL: false,
    buttonR: false,
    timestamp: 0,
};

const GamepadManagerContext = createContext<GamepadManagerContextType>({
    state: defaults,
    setState: () => {},
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
    useEffect(() => {
        gamepadLoop(sendToServer, setState);
    }, []);

    useEffect(() => {
        function handleKeyDown(event: KeyboardEvent) {
            const newState = { ...defaultsNormalized };
            if (event.key === "ArrowUp" || event.key === "w") {
                newState.y1 = -1;
                newState.y2 = -1;
            }
            if (event.key === "ArrowDown" || event.key === "s") {
                newState.y1 = 1;
                newState.y2 = 1;
            }
            if (event.key === "d") {
                newState.y1 = -1;
                newState.y2 = 1;
            }
            if (event.key === "a") {
                newState.y1 = 1;
                newState.y2 = -1;
            }
            if (event.key === "z" || event.key === "q") {
                newState.buttonL = !newState.buttonL;
            }
            if (event.key === "x" || event.key === "e") {
                newState.buttonR = !newState.buttonR;
            }
            newState.timestamp = Date.now();
            sendToServer("controls", newState);
            setState(newState);
        }

        function handleKeyUp(event: KeyboardEvent) {
            if (defaults !== state) {
                sendToServer(
                    "controls", 
                    {
                        ...defaultsNormalized,
                        timestamp: Date.now(),
                    }
                );
            }
            setState(defaults);
        }
        window.addEventListener("keydown", handleKeyDown);
        window.addEventListener("keyup", handleKeyUp);

        return () => {
            window.removeEventListener("keydown", handleKeyDown);
            window.removeEventListener("keyup", handleKeyUp);
        };
    }, []);
    return (
        <GamepadManagerContext.Provider value={{ state, setState }}>
            {children}
        </GamepadManagerContext.Provider>
    );
}
