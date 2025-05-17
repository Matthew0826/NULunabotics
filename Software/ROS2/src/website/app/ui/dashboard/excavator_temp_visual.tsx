import { useWebSocketContext } from "@/app/lib/web-socket-context";
import PercentViewer from "./percent-viewer";
import { useEffect, useState } from "react";

export default function ExcavatorVisual() {
    const { messages, sendToServer } = useWebSocketContext();
    const [excavatorState, setExcavatorState] = useState<number[]>([]);
    const [excavatorSpeed, setExcavatorSpeed] = useState<number[]>([]);
    const [distanceSensor, setDistanceSensor] = useState<number>(0.0);
    useEffect(() => {
        if (messages.length == 0) return;
        const excavatorMessage = messages[messages.length - 1];
        if (excavatorMessage.type === "excavator_percent") {
            const lifter = excavatorMessage?.message?.excavator_lifter_percent || 0.0;
            const actuator = excavatorMessage?.message?.actuator_percent || 0.0;
            setExcavatorState([lifter, actuator]);
        } else if (excavatorMessage.type === "distance_sensor") {
            const distancePercent = (excavatorMessage?.message?.data || 0.0) / 20.0;
            setDistanceSensor(Math.min(distancePercent, 1.0));
        } else if (excavatorMessage.type === "excavator") {
            const lifter = excavatorMessage?.message?.excavator_lifter_speed || 0.0;
            const conveyor = excavatorMessage?.message?.conveyor_speed || 0.0;
            const actuator = excavatorMessage?.message?.actuator_speed || 0.0;
            setExcavatorSpeed([Math.abs(lifter), Math.abs(conveyor), Math.abs(actuator)]);
        }
    }, [messages]);
    return (
        <div className="flex flex-col gap-4">
            <PercentViewer description="Excavator Lifter" values={[excavatorState[0]]} />
            <PercentViewer description="Excavator Distance Sensor" values={[distanceSensor]} />
            <PercentViewer description="Bin Actuator" values={[excavatorState[1]]} />
            <PercentViewer description="Excavator, Conveyor, and Bin Speeds" values={excavatorSpeed} />
        </div>
    )
}