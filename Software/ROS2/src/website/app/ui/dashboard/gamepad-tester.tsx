"use client";

import DirectionalDisplay from "@/app/ui/dashboard/components/directional-display";
import {useGamepadManagerContext} from "@/app/contexts/gamepad-context";

export default function GamepadTester() {
    const { state } = useGamepadManagerContext();
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
