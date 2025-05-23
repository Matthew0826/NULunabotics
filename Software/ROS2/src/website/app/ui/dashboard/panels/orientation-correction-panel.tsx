"use client";

import { useEffect, useMemo, useState } from "react";
import Slider from "../components/slider";
import { useWebSocketContext } from "@/app/contexts/web-socket-context";

export default function OrientationCorrectionPanel() {
    const [previousOrientationCorrection, setPreviousOrientationCorrection] = useState(0.0);
    const [orientationCorrection, setOrientationCorrection] = useState(0.0);
    const { sendToServer } = useWebSocketContext();

    const [isClient, setIsClient] = useState(false);
    useEffect(() => {
        setIsClient(true);
    }, []);
    if (!isClient) {
        return <div />;
    }

    const handleOrientationCorrection = (correction: number) => {
        sendToServer("orientationCorrection", {
            orientationCorrection: (orientationCorrection - correction),
        });
        setOrientationCorrection(correction);
    };
    return (
        <Slider labels={[0.0, 90.0, 180.0, 270.0, 360.0]} value={orientationCorrection} setValue={handleOrientationCorrection} min={0} max={360} step={1} />
    )
}
