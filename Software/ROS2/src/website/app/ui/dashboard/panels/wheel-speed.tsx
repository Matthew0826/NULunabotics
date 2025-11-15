"use client";

import { useGamepadManagerContext } from "@/app/contexts/gamepad-context";
import Slider from "../components/slider";
import { useEffect, useState } from "react";

export default function WheelSpeedPanel() {
    const { state, setState, speed, setSpeed } = useGamepadManagerContext();

    const [isClient, setIsClient] = useState(false);
    useEffect(() => {
        setIsClient(true);
    }, []);
    if (!isClient) {
        return <div />;
    }

    return <Slider labels={[0.0, 0.5, 1.0]} value={speed} setValue={setSpeed} />
}
