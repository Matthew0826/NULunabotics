import { useEffect, useState } from "react";
import { defaults, useGamepadManagerContext } from "../ui/dashboard/gamepad-state-provider";
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

export function useKeyboardController() {
    const { messages, sendToServer } = useWebSocketContext();
    const { state, setState } = useGamepadManagerContext();
    const [speed, setSpeed] = useState(0.5);
    useEffect(() => {
        function handleKeyDown(event: KeyboardEvent) {
            const newState = { ...defaultsNormalized };
            if (event.key === "ArrowUp" || event.key === "w") {
                newState.y1 = speed;
                newState.y2 = speed;
            }
            if (event.key === "ArrowDown" || event.key === "s") {
                newState.y1 = -speed;
                newState.y2 = -speed;
            }
            if (event.key === "d") {
                newState.y1 = speed;
                newState.y2 = -speed;
            }
            if (event.key === "a") {
                newState.y1 = -speed;
                newState.y2 = speed;
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
    return [speed, setSpeed];
}