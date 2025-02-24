"use client";

import { gamepadLoop, moveArrow } from "@/app/lib/utils";
import { useWebSocketContext } from "@/app/socket/web-socket-context";
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
};

type GamepadManagerContextType = {
    state: GamepadState;
    setState: React.Dispatch<React.SetStateAction<GamepadState>>;
};

const GamepadManagerContext = createContext<GamepadManagerContextType>({
    state: {
        x1: 100,
        y1: 100,
        x2: 100,
        y2: 100,
        buttonL: false,
        buttonR: false,
    },
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
    });
    useEffect(() => {
        gamepadLoop(sendToServer, setState);
    }, []);

    useEffect(() => {
        function handleKeyDown(event: KeyboardEvent) {
            const newState = {
                x1: state.x1 > 0 ? 1 : state.x1 < 0 ? -1 : 0,
                y1: state.y1 > 0 ? 1 : state.y1 < 0 ? -1 : 0,
                x2: 100,
                y2: 100,
                buttonL: false,
                buttonR: false,
            };
            if (event.key === "ArrowUp" || event.key === "w") {
                newState.y1 = -1;
            }
            if (event.key === "ArrowDown" || event.key === "s") {
                newState.y1 = 1;
            }
            if (event.key === "ArrowLeft" || event.key === "a") {
                newState.x1 = -1;
            }
            if (event.key === "ArrowRight" || event.key === "d") {
                newState.x1 = 1;
            }
            if (event.key === "z" || event.key === "q") {
                newState.buttonL = !newState.buttonL;
            }
            if (event.key === "x" || event.key === "e") {
                newState.buttonR = !newState.buttonR;
            }
            const { endX, endY } = moveArrow(newState.x1, newState.y1);
            newState.x1 = endX;
            newState.y1 = endY;
            setState(newState);
        }

        function handleKeyUp(event: KeyboardEvent) {
            setState({
                x1: 100,
                y1: 100,
                x2: 100,
                y2: 100,
                buttonL: false,
                buttonR: false,
            });
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
