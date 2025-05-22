import {GamepadState} from "@/app/contexts/gamepad-context";
import React from "react";

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
    sendToServer: (messageType: string, message: any) => void,
    setState: React.Dispatch<React.SetStateAction<GamepadState>>,
    speed: number = 0.5
) {
    let start: number;
    let lastSent = 0; // Throttle timestamp
    console.log("Beginning gamepad loop.");

    function step(timestamp: number) {
        if (start === undefined) {
            start = timestamp;
        }

        var gamepads = navigator.getGamepads();
        // console.log(gamepads);
        var gamepad: Gamepad | null = gamepads[0];
        if (gamepad) {
            console.log(gamepad.axes);
        } else {
            requestAnimationFrame(step);
            return;
        }

        // gamepad.buttons.forEach((button, index) => {
        //     if (button.pressed) {
        //         console.log("Button " + index + " pressed");
        //         if (index === 4) {
        //             gamepad?.vibrationActuator.playEffect("dual-rumble", {
        //                 startDelay: 0,
        //                 duration: 100,
        //                 weakMagnitude: 0.0,
        //                 strongMagnitude: 1.0,
        //             });
        //         } else if (index === 5) {
        //             gamepad?.vibrationActuator.playEffect("dual-rumble", {
        //                 startDelay: 0,
        //                 duration: 100,
        //                 weakMagnitude: 1.0,
        //                 strongMagnitude: 0.0,
        //             });
        //         }
        //     }
        // });

        var x1 = gamepad.axes[0];
        var y1 = gamepad.axes[1];
        var x2 = gamepad.axes[2];
        var y2 = gamepad.axes[3];
        const now = Date.now();

        const newData = {
            x: x1,
            y: -y1,
            conveyorSpeed: gamepad.buttons[5].pressed ? speed : 0.0,
            isActuator: !gamepad.buttons[4].pressed,
            actuatorPower: y2 * speed,
            timestamp: now
        };

        // Throttled. if a message was sent in the last 100ms, don't send another one until
        // the 100ms is up. If a message was sent over that, send a new one.
        if (now - lastSent >= 100) {
            setState(newData);
            sendToServer("controls", newData);
            lastSent = now;
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
