import { Dispatch, SetStateAction, useEffect, useRef, useState } from "react";
import {useWebSocketContext} from "@/app/contexts/web-socket-context";
import {useGamepadManagerContext} from "@/app/contexts/gamepad-context";

const defaultsNormalized = {
    x: 0,
    y: 0,
    actuatorPower: 0,
    isActuator: false,
    conveyorSpeed: 0.0,
    timestamp: 0,
};

function statesAreEqual(a: any, b: any): boolean {
    return Object.keys(a).every(k => a[k] == b[k]) &&
        Object.keys(b).every(k => a[k] == b[k]);
}

export function useKeyboardController(): [number, Dispatch<SetStateAction<number>>] {
    const { sendToServer } = useWebSocketContext();
    const { state, setState } = useGamepadManagerContext();
    const [speed, setSpeed] = useState(0.5);
    const [flipped, setFlipped] = useState(false);
    const speedRef = useRef(0.5);
    useEffect(() => {
        speedRef.current = speed;
        function handleKeyDown(event: KeyboardEvent) {
            const newState = { ...defaultsNormalized };
            if (event.key === "ArrowUp" || event.key === "w") {
                newState.x = 0;
                newState.y = speedRef.current;
            }
            if (event.key === "ArrowDown" || event.key === "s") {
                newState.x = 0;
                newState.y = -speedRef.current;
            }
            if (event.key === "d") {
                newState.x = speedRef.current;
                newState.y = 0;
            }
            if (event.key === "a") {
                newState.x = -speedRef.current;
                newState.y = 0;
            }
            if (event.key === ",") {
                newState.actuatorPower = -speedRef.current;
            }
            if (event.key === ".") {
                newState.actuatorPower = speedRef.current;
            }
            let flipActuator = false;
            if (event.key == "q") {
                flipActuator = true;
                setFlipped((prev) => !prev);
            }
            if (event.key == "e") {
                newState.conveyorSpeed = speedRef.current;
            }
            newState.timestamp = Date.now();


            sendToServer("controls", { ...newState, isActuator: flipped });
            setState(newState);
        }

        function handleKeyUp(event: KeyboardEvent) {
            if (state != defaultsNormalized) {
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
