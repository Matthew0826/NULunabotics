import { Point } from "@/app/lib/ros2";
import { Message, useWebSocketContext } from "@/app/lib/web-socket-context";
import Image from "next/image";
import { JSX, useEffect } from "react";
import { useState } from "react";
import { MAP_HEIGHT, ObstacleType } from "../map/map";
import Obstacle from "../map/obstacle";

function interpolateColor(color1: number[], color2: number[], factor: number) {
    const result = color1.slice();
    for (let i = 0; i < 3; i++) {
        result[i] = Math.round(result[i] + factor * (color2[i] - color1[i]));
    }
    return `rgb(${result[0]}, ${result[1]}, ${result[2]})`;
}

const previousLidarData: Point[][] = [];
const previousLidarDataMaxSize = 20;
const THRESHOLD_DEGS = 0.25;
const THRESHOLD_RADS = THRESHOLD_DEGS * 2.0 * (Math.PI / 180.0);

export default function LidarVisual() {
    const { messages, sendToServer } = useWebSocketContext();
    const [lidarData, setLidarData] = useState<Point[]>([]);
    const [obstacles, setObstacles] = useState<ObstacleType[]>([]);
    useEffect(() => {
        if (messages.length == 0) return;
        const lidarMessage = messages[messages.length - 1];
        if (lidarMessage.type === "lidar") {
            setLidarData(lidarMessage?.message || []);
            // GETS POINT IN RADIANS!!
            // HAZEM SAYS ITS NOT DEGREES!!
            previousLidarData.push(lidarMessage?.message || []);
            if (previousLidarData.length > previousLidarDataMaxSize) {
                previousLidarData.shift();
            }
        }
        const obstaclesMessages = [...new Set(messages
            .filter((message: Message) => message.type === "obstacles")
            .map((message: Message) => message.message)
            .flat()
            .map((newObstacle: any) => {
                return { x: newObstacle.position.x, y: newObstacle.position.y, radius: newObstacle.radius, isHole: true }
            }))];
        // console.log(obstaclesMessages);

        setObstacles(obstaclesMessages);
    }, [messages]);
    const getMaxDistance = () => {
        return 500; //Math.max(...lidarData.map((point) => point.distance));
    };

    // Angle is in radians
    const getAveragePreviousPoint = (angle: number) => {
        // Find the points in the previousLidarData that are closest to the given angle
        const points = previousLidarData
            .map((data) =>
                data.find((point) => Math.abs(point.angle - angle) < THRESHOLD_RADS)
            )
            .filter((point) => point !== undefined);
        // Find average distance and angle
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
                            [0, 255, 0],
                            [255, 0, 0],
                            //   point.weight / 256
                            // how far away from moving average
                            Math.min(
                                1.0,
                                getAveragePreviousPoint(
                                    Math.floor(point.angle)
                                ).distance / 200.0
                            )
                        )
                        : "blue",
                    left: `${50 + 50 * (point.distance / getMaxDistance()) * Math.cos(point.angle)}%`,
                    bottom: `${50 + 50 * (point.distance / getMaxDistance()) * Math.sin(point.angle)}%`,
                }}
                key={index}
            />
        );
    }
    return (
        <div className="relative w-[35vh] h-[35vh] m-2">
            {/*lidarData.map((point, index) =>
                getDivFromLidar(point, index, true)
            )*/}
            {previousLidarData.length > 0 && (
                Array.from({ length: 360 / THRESHOLD_DEGS }).map((_, index) =>
                    getDivFromLidar(
                        getAveragePreviousPoint((index * THRESHOLD_DEGS) / 180.0 * Math.PI),
                        index,
                        false
                    )
                )
            )}
            <img
                src="/lidar_icon.svg"
                alt="Lidar Icon"
                className="absolute w-[10%] h-auto top-1/2 left-1/2 -translate-y-1/2 -translate-x-1/2"
            />
            {obstacles.map((obstacle, index) => (
                <Obstacle
                    key={index}
                    x={obstacle.x * 100}
                    y={obstacle.y * 100}
                    radius={obstacle.radius * 100}
                    isHole={obstacle.isHole}
                    parentHeight={getMaxDistance()}
                    parentWidth={getMaxDistance()}
                />
            ))}
        </div>
    );
}
