import { Reference, RefObject } from "react";
import WebSocket from "ws";
import { GamepadState } from "../ui/dashboard/gamepad-state-provider";

function setupGamePad() {
    window.addEventListener("gamepadconnected", (event) => {
        console.log("A gamepad connected:");
        console.log(event.gamepad);
    });

    window.addEventListener("gamepaddisconnected", (event) => {
        console.log("A gamepad disconnected:");
        console.log(event.gamepad);
    });
}

export function normalizedVectorToPixels(
    x: number,
    y: number
): { endX: number; endY: number } {
    const arrowLength = Math.sqrt(x * x + y * y);
    if (arrowLength > 1) {
        x /= arrowLength;
        y /= arrowLength;
    }
    const endX = 100 + x * 90;
    const endY = 100 + y * 90;
    return { endX, endY };
}

export var gamepadData = {};

export function gamepadLoop(
    sendToServer: (data: string) => void,
    setState: React.Dispatch<React.SetStateAction<GamepadState>>
) {
    let start: number;
    console.log("Beginning gamepad loop.");
    function step(timestamp: number) {
        if (start === undefined) {
            start = timestamp;
        }
        const elapsed = timestamp - start;

        var gamepads = navigator.getGamepads();
        // console.log(gamepads);
        var gamepad: Gamepad | null = gamepads[0];
        if (gamepad) {
            console.log(gamepad.axes);
        } else {
            requestAnimationFrame(step);
            return;
        }
        // buttonL.classList.remove("black-bg");
        // buttonR.classList.remove("black-bg");
        gamepad.buttons.forEach((button, index) => {
            if (button.pressed) {
                console.log("Button " + index + " pressed");
                if (index === 4) {
                    // buttonL.classList.add("black-bg");
                    gamepad?.vibrationActuator.playEffect("dual-rumble", {
                        startDelay: 0,
                        duration: 100,
                        weakMagnitude: 0.0,
                        strongMagnitude: 1.0,
                    });
                } else if (index === 5) {
                    // buttonR.classList.add("black-bg");
                    gamepad?.vibrationActuator.playEffect("dual-rumble", {
                        startDelay: 0,
                        duration: 100,
                        weakMagnitude: 1.0,
                        strongMagnitude: 0.0,
                    });
                }
            }
        });

        var x1 = gamepad.axes[0];
        var y1 = gamepad.axes[1];
        var x2 = gamepad.axes[2];
        var y2 = gamepad.axes[3];

        var newData: GamepadState = {
            x1,
            y1,
            x2,
            y2,
            buttonL: gamepad.buttons[4].pressed,
            buttonR: gamepad.buttons[5].pressed,
            timestamp: 0,
        };

        if (gamepadData !== newData) {
            gamepadData = newData;
            sendToServer(JSON.stringify({ ...newData, timestamp: Date.now() }));
            const end1 = normalizedVectorToPixels(x1, y1);
            const end2 = normalizedVectorToPixels(x2, y2);
            newData.x1 = end1.endX;
            newData.y1 = end1.endY;
            newData.x2 = end2.endX;
            newData.y2 = end2.endY;
            setState(newData);
        }

        requestAnimationFrame(step);
    }

    requestAnimationFrame(step);
}

export type Dataset = {
    label: string;
    color: string;
    data: number[];
};
