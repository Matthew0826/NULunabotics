import { Dispatch, SetStateAction, useEffect, useReducer, useRef, useState } from "react";
import { defaults, GamepadState, useGamepadManagerContext } from "../ui/dashboard/gamepad-state-provider";
import { useWebSocketContext } from "./web-socket-context";

const defaultsNormalized = {
    x1: 0,
    y1: 0,
    x2: 0,
    y2: 0,
    buttonL: false,
    buttonR: false,
    timestamp: 0,
};

function statesAreEqual(a: any, b: any): boolean {
    return Object.keys(a).every(k => a[k] === b[k]) &&
        Object.keys(b).every(k => a[k] === b[k]);
}

export function useKeyboardController(): [number, Dispatch<SetStateAction<number>>] {
    const { messages, sendToServer } = useWebSocketContext();
    const { state, setState } = useGamepadManagerContext();
    const [speed, setSpeed] = useState(0.5);
    const speedRed = useRef(0.5);
    useEffect(() => {
        speedRed.current = speed;
        function handleKeyDown(event: KeyboardEvent) {
            const newState = { ...defaultsNormalized };
            if (event.key === "ArrowUp" || event.key === "w") {
                newState.y1 = speedRed.current;
                newState.y2 = speedRed.current;
            }
            if (event.key === "ArrowDown" || event.key === "s") {
                newState.y1 = -speedRed.current;
                newState.y2 = -speedRed.current;
            }
            if (event.key === "d") {
                newState.y1 = speedRed.current;
                newState.y2 = -speedRed.current;
            }
            if (event.key === "a") {
                newState.y1 = -speedRed.current;
                newState.y2 = speedRed.current;
            }
            if (event.key === "z" || event.key === "q") {
                newState.buttonL = true;//!newState.buttonL;
            }
            if (event.key === "x" || event.key === "e") {
                newState.buttonR = true; !newState.buttonR;
            }
            newState.timestamp = Date.now();
            sendToServer("controls", newState);
            setState(newState);
        }

        function handleKeyUp(event: KeyboardEvent) {
            if (!statesAreEqual(state, defaultsNormalized)) {
                sendToServer(
                    "controls",
                    {
                        ...defaultsNormalized,
                        timestamp: Date.now(),
                    }
                );
            }
            setState(defaultsNormalized);
        }
        window.addEventListener("keydown", handleKeyDown);
        window.addEventListener("keyup", handleKeyUp);

        return () => {
            window.removeEventListener("keydown", handleKeyDown);
            window.removeEventListener("keyup", handleKeyUp);
        };
    }, [sendToServer, setState, speed]);
    return [speed, setSpeed];
}