import { useWebSocketContext } from "@/app/socket/web-socket-context";
import { useEffect } from "react";
import { useState } from "react";

export default function LidarVisual() {
    const { messages, sendToServer } = useWebSocketContext();
    const [lidarData, setLidarData] = useState<
        { distance: number; angle: number }[]
    >([]);
    useEffect(() => {
        if (messages.length == 0) return;
        const lidarMessage = messages.findLast(
            (message) => JSON.parse(message)[0].graph === "Lidar"
        );
        setLidarData(
            (lidarMessage ? JSON.parse(lidarMessage)[0] : null)?.newData || []
        );
    }, [messages]);
    const getMaxDistance = () => {
        return Math.max(...lidarData.map((point) => point.distance));
    };
    return (
        <div className="relative w-full aspect-[1/1] m-2">
            {lidarData.map((point, index) => (
                <div
                    className="absolute rounded-full -translate-y-1/2 -translate-x-1/2"
                    style={{
                        width: 5,
                        height: 5,
                        backgroundColor: "red",
                        left: `${50 + 50 * (point.distance / getMaxDistance()) * Math.cos(point.angle)}%`,
                        bottom: `${50 + 50 * (point.distance / getMaxDistance()) * Math.sin(point.angle)}%`,
                    }}
                    key={index}
                />
            ))}
        </div>
    );
}
