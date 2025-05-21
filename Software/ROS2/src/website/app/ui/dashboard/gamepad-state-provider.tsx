"use client";

import { useKeyboardController } from "@/app/lib/keyboard-controller";
import { gamepadLoop, normalizedVectorToPixels } from "@/app/lib/utils";
import { useWebSocketContext } from "@/app/lib/web-socket-context";
import { useContext, useState } from "react";
import { createContext } from "react";
import { useEffect } from "react";

export type GamepadState = {
    x: number;
    y: number;
    actuatorPower: number;
    isActuator: boolean;
    conveyorSpeed: number;
    timestamp: number;
};

type GamepadManagerContextType = {
    state: GamepadState;
    setState: React.Dispatch<React.SetStateAction<GamepadState>>;
    speed: number;
    setSpeed: React.Dispatch<React.SetStateAction<number>>;
};

export const defaults = {
    x: 0,
    y: 0,
    actuatorPower: 0,
    isActuator: false,
    conveyorSpeed: 0.0,
    timestamp: 0,
};


const GamepadManagerContext = createContext<GamepadManagerContextType>({
    state: defaults,
    setState: () => { },
    speed: 0,
    setSpeed: () => { },
});

export const useGamepadManagerContext = () => useContext(GamepadManagerContext);

export default function GamepadStateProvider({
    children,
}: {
    children: React.ReactNode;
}) {
    const { messages, sendToServer } = useWebSocketContext();
    const [state, setState] = useState<GamepadState>(defaults);
    const [speed, setSpeed] = useKeyboardController();

    useEffect(() => {
        gamepadLoop(sendToServer, setState, speed);
    }, [speed]);
    return (
        <GamepadManagerContext.Provider value={{ state, setState, speed, setSpeed }}>
            {children}
        </GamepadManagerContext.Provider>
    );
}
