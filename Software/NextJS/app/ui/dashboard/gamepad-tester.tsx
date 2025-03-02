"use client";

import DirectionalDisplay from "./directional-display";
import { useGamepadManagerContext } from "./gamepad-state-provider";

export default function GamepadTester() {
    const { state, setState } = useGamepadManagerContext();
    return (
        <>
            <DirectionalDisplay
                x={state.x1}
                y={state.y1}
                buttonPressed={state.buttonL}
            />
            <DirectionalDisplay
                x={state.x2}
                y={state.y2}
                buttonPressed={state.buttonR}
            />
        </>
    );
}
