import { Reference, RefObject } from "react";
import WebSocket from "ws";

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

function moveArrow(arrowLine: any, arrowCircle: any, x: number, y: number) {
    const arrowLength = Math.sqrt(x * x + y * y);
    if (arrowLength > 1) {
        x /= arrowLength;
        y /= arrowLength;
    }
    const endX = 100 + x * 90;
    const endY = 100 + y * 90;
    arrowLine.setAttribute("x2", endX);
    arrowLine.setAttribute("y2", endY);
    arrowCircle.setAttribute("cx", endX);
    arrowCircle.setAttribute("cy", endY);
}

export var gamepadData = {};

export function gamepadLoop(
    buttonL: any,
    buttonR: any,
    arrowLine1: any,
    arrowCircle1: any,
    arrowLine2: any,
    arrowCircle2: any,
    socketRef: RefObject<WebSocket | null>
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
        buttonL.classList.remove("black-bg");
        buttonR.classList.remove("black-bg");
        gamepad.buttons.forEach((button, index) => {
            if (button.pressed) {
                console.log("Button " + index + " pressed");
                if (index === 4) {
                    buttonL.classList.add("black-bg");
                    gamepad?.vibrationActuator.playEffect("dual-rumble", {
                        startDelay: 0,
                        duration: 100,
                        weakMagnitude: 0.0,
                        strongMagnitude: 1.0,
                    });
                } else if (index === 5) {
                    buttonR.classList.add("black-bg");
                    gamepad?.vibrationActuator.playEffect("dual-rumble", {
                        startDelay: 0,
                        duration: 100,
                        weakMagnitude: 1.0,
                        strongMagnitude: 0.0,
                    });
                }
            }
        });

        var x = gamepad.axes[0];
        var y = gamepad.axes[1];
        var x2 = gamepad.axes[2];
        var y2 = gamepad.axes[3];

        var newData = {
            leftStick: { x, y },
            rightStick: { x2, y2 },
            buttonL: gamepad.buttons[4].pressed,
            buttonR: gamepad.buttons[5].pressed,
        };
        if (gamepadData !== newData) {
            gamepadData = newData;
        }
        socketRef.current?.send(JSON.stringify(gamepadData));

        moveArrow(arrowLine2, arrowCircle1, x, y);
        moveArrow(arrowLine1, arrowCircle2, x2, y2);
        requestAnimationFrame(step);
    }

    requestAnimationFrame(step);
}

export type Dataset = {
    label: string;
    color: string;
    data: number[];
};
