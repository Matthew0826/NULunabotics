import { useWebSocketContext } from "@/app/lib/web-socket-context";
import Obstacle from "./obstacle";
import RobotPath, { Point } from "./robot-path";
import { useEffect, useState } from "react";

export const MAP_WIDTH = 5.48; // meters
export const MAP_OBSTACLES_ZONE_HEIGHT = 2.44; // meters
export const MAP_HEIGHT = 4.87; // meters
export const COLUMN_WIDTH = 0.8; // meters

type ObstacleType = {
    x: number;
    y: number;
    radius: number;
    isHole: boolean;
};

// Note: all obstacles and paths are in centimeters
export default function Map() {
    const { messages, sendToServer } = useWebSocketContext();
    const [pathData, setPathData] = useState<Point[]>([]);
    const [obstacles, setObstacles] = useState<ObstacleType[]>([]);
    useEffect(() => {
        if (messages.length == 0) return;
        const pathMessage = JSON.parse(messages[messages.length - 1]).find(
            (message: any) => message.graph === "Path"
        );
        const data = (pathMessage?.newData || []).map((point: any) => ([point.x, point.y]));
        // console.log("Path data", data);
        setPathData(data);
        const obstaclesMessages = [...new Set(messages.map((message: any) => JSON.parse(message)[0])
            .filter((message: any) => message.graph === "Obstacles")
            .map((message: any) => message.newData)
            .map((newObstacle: any) => {
                return { x: newObstacle.position.x, y: newObstacle.position.y, radius: newObstacle.radius, isHole: true }
            }))];

        setObstacles(obstaclesMessages);
    }, [messages]);
    return (
        <div style={{ padding: "1vw" }}>
            <div
                className="flex flex-wrap relative mx-auto gap-0"
                style={{
                    aspectRatio: `${MAP_WIDTH}/${MAP_HEIGHT}`,
                    maxHeight: "82vh",
                }}
            >
                <div
                    className="relative w-full border-4 border-slate-800 border-b-0"
                    style={{
                        height: `${(100 * MAP_OBSTACLES_ZONE_HEIGHT) / MAP_HEIGHT
                            }%`,
                    }}
                >
                    <p className="absolute t-0 left-1/2 transform -translate-x-1/2 text-center text-sm">
                        Obstacles
                    </p>
                    {/* <div
                        className="absolute bg-slate-300 top-1/2 right-[-4px] transform -translate-y-1/2 border-4 border-slate-800 flex flex-col justify-center items-center"
                        style={{
                            width:
                                Math.floor(
                                    (100 * COLUMN_WIDTH) /
                                        MAP_OBSTACLES_ZONE_WIDTH
                                ) + "%",
                            height: (100 * COLUMN_WIDTH) / MAP_HEIGHT + "%",
                        }}
                    >
                        <p className="text-center text-sm">Column</p>
                    </div> */}
                    <div
                        className={`absolute right-0 aspect-square border-4 border-r-0 border-t-0 border-slate-800 flex flex-col justify-center items-center`}
                        style={{
                            height: `${Math.floor(
                                (100 * 2) /
                                (MAP_HEIGHT - MAP_OBSTACLES_ZONE_HEIGHT)
                            )}%`,
                        }}
                    >
                        <p className="text-center text-sm">Start</p>
                    </div>
                </div>
                <div
                    className="flex w-full"
                    style={{
                        height: `${(100 * (MAP_HEIGHT - MAP_OBSTACLES_ZONE_HEIGHT)) /
                            MAP_HEIGHT
                            }%`,
                    }}
                >
                    <div className="w-1/2 border-4 border-slate-800">
                        <p className="text-center text-sm">Excavation</p>
                    </div>
                    <div className="w-1/2 border-4 border-slate-800 border-l-0">
                        <p className="text-center text-sm">Construction</p>
                    </div>
                </div>
                {obstacles.map((obstacle, index) => (
                    <Obstacle
                        key={index}
                        x={obstacle.x}
                        y={obstacle.y}
                        radius={obstacle.radius}
                        isHole={obstacle.isHole}
                    />
                ))}
                <RobotPath path={pathData} />
            </div>
        </div>
    );
}
