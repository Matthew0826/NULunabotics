import Obstacle from "./obstacle";
import RobotPath from "./robot-path";
import { useEffect, useRef, useState } from "react";
import Robot from "./robot";
import {useRobotContext} from "@/app/contexts/robot-context";
import {useWebSocketContext} from "@/app/contexts/web-socket-context";
import {MapPoint} from "@/app/types/map-objects";

export const MAP_WIDTH = 5.48; // meters
export const MAP_OBSTACLES_ZONE_HEIGHT = 2.44; // meters
export const MAP_HEIGHT = 4.87; // meters
export const COLUMN_WIDTH = 0.8; // meters


// Note: all obstacles and paths are in centimeters
export default function Map() {
    const { messages, sendToServer } = useWebSocketContext();
    const [pathData, setPathData] = useState<MapPoint[]>([]);
    const [odometryPathData, setOdometryPathData] = useState<MapPoint[]>([]);

    const { robot, obstacles } = useRobotContext();

    useEffect(() => {
        if (messages.length == 0) {
            // in the case of a reset, clear the path data
            setPathData([]);
            setOdometryPathData([]);
            return;
        }
        const lastMessage = messages[messages.length - 1];
        if (lastMessage.type === "path") {
            const data = (lastMessage?.message || []).map((point) => ([point.x, point.y] as MapPoint));
            // console.log("Path data", data);
            setPathData(data);
        } else if (lastMessage.type === "odometry_path") {
            const data = (lastMessage?.message || []).map((point) => ([point.x, point.y] as MapPoint));
            // console.log("Odometry path data", data);
            setOdometryPathData(data);
        }

    }, [messages]);

    // used to focus the map div when the page loads
    // so that keyboard controls work
    const divRef = useRef<HTMLDivElement>(null);

    useEffect(() => {
        divRef?.current?.focus();
    }, []);

    return (
        <div style={{ padding: "1vw" }} ref={divRef}>
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
                            height: `${(200) / (MAP_HEIGHT - MAP_OBSTACLES_ZONE_HEIGHT)}%`,
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
                    <div className="relative w-1/2 border-4 border-slate-800 border-l-0">
                        <p className="text-center text-sm">Construction</p>

                        <div
                            className={`absolute border-4 border-slate-800 flex flex-col justify-center items-center`}
                            style={{
                                height: `${(200) / (MAP_OBSTACLES_ZONE_HEIGHT)}%`,
                                width: `${(70) / (MAP_OBSTACLES_ZONE_HEIGHT)}%`,
                                transform: `translate(0%, -50%)`,
                                top: "50%",
                                right: `${(50) / (MAP_WIDTH / 2)}%`
                            }}
                        >
                            <p className="text-center text-sm">Berm</p>
                        </div>
                    </div>
                </div>
                {obstacles.map((obstacle, index) => (
                    <Obstacle
                        key={index}
                        x={obstacle.x}
                        y={obstacle.y}
                        radius={obstacle.radius}
                        isHole={obstacle.isHole}
                        parentHeight={MAP_HEIGHT}
                        parentWidth={MAP_WIDTH}
                    />
                ))}
                <Robot x={robot.x} y={robot.y} width={robot.width} height={robot.height} rotation={robot.rotation} confidenceRect={robot.posConfidenceRect} />
                <RobotPath path={pathData} odometryPath={odometryPathData} robot={robot} />
            </div>
        </div>
    );
}
