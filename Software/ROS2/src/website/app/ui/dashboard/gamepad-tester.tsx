"use client";

import DirectionalDisplay from "./directional-display";
import { useGamepadManagerContext } from "./gamepad-state-provider";

export default function GamepadTester() {
    const { state, setState, speed, setSpeed } = useGamepadManagerContext();
    return (
        <>
            <DirectionalDisplay
                x={state.x}
                y={state.y}
                buttonPressed={state.isActuator}
            />
            <DirectionalDisplay
                x={0}
                y={state.actuatorPower}
                buttonPressed={state.conveyorSpeed > 0}
            />
        </>
    );
}
