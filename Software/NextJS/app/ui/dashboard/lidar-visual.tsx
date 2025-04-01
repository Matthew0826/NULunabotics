import { Point } from "@/app/lib/ros2";
import { useWebSocketContext } from "@/app/lib/web-socket-context";
import Image from "next/image";
import { JSX, useEffect } from "react";
import { useState } from "react";

function interpolateColor(color1: number[], color2: number[], factor: number) {
    const result = color1.slice();
    for (let i = 0; i < 3; i++) {
        result[i] = Math.round(result[i] + factor * (color2[i] - color1[i]));
    }
    return `rgb(${result[0]}, ${result[1]}, ${result[2]})`;
}

const previousLidarData: Point[][] = [];
const previousLidarDataMaxSize = 20;

export default function LidarVisual() {
    const { messages, sendToServer } = useWebSocketContext();
    const [lidarData, setLidarData] = useState<Point[]>([]);
    useEffect(() => {
        if (messages.length == 0) return;
        const lidarMessage = JSON.parse(messages[messages.length - 1]).find(
            (message: any) => message.graph === "Lidar"
        );
        setLidarData(lidarMessage?.newData || []);
        previousLidarData.push(lidarMessage?.newData || []);
        if (previousLidarData.length > previousLidarDataMaxSize) {
            previousLidarData.shift();
        }
    }, [messages]);
    const getMaxDistance = () => {
        return 1000; //Math.max(...lidarData.map((point) => point.distance));
    };

    const getAveragePreviousPoint = (angle: number) => {
        const points = previousLidarData
            .map((data) =>
                data.find((point) => Math.floor(point.angle) == angle)
            ) // Does floating point rounding weirdness mess this up?
            .filter((point) => point !== undefined);
        const averageDistance =
            points.reduce((sum, point) => sum + point.distance, 0) /
            points.length;
        const averageAngle =
            points.reduce((sum, point) => sum + point.angle, 0) / points.length;
        return {
            distance: averageDistance,
            angle: averageAngle,
            weight: -1, // Not relevant for the moving average
        };
    };

    function getDivFromLidar(
        point: Point,
        index: number,
        doInterpolateColor: boolean
    ): JSX.Element {
        return (
            <div
                className="absolute rounded-full -translate-y-1/2 -translate-x-1/2"
                style={{
                    width: 4,
                    height: 4,
                    backgroundColor: doInterpolateColor
                        ? interpolateColor(
                              [255, 0, 0],
                              [0, 255, 0],
                              point.weight / 256
                          )
                        : "blue",
                    left: `${50 + 50 * (point.distance / getMaxDistance()) * Math.cos(point.angle)}%`,
                    bottom: `${50 + 50 * (point.distance / getMaxDistance()) * -Math.sin(point.angle)}%`,
                }}
                key={index}
            />
        );
    }
    return (
        <div className="relative w-[35vh] h-[35vh] m-2">
            {lidarData.map((point, index) =>
                getDivFromLidar(point, index, true)
            )}
            {previousLidarData.length > 0 ? (
                Array.from({ length: 360 }).map((_, index) =>
                    getDivFromLidar(
                        getAveragePreviousPoint(index),
                        index,
                        false
                    )
                )
            ) : (
                <></>
            )}
            <img
                src="/lidar_icon.svg"
                alt="Lidar Icon"
                className="absolute w-[10%] h-auto top-1/2 left-1/2 -translate-y-1/2 -translate-x-1/2"
            />
        </div>
    );
}
