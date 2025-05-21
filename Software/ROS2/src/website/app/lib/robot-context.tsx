"use client";

import {createContext, ReactNode, useContext, useEffect, useState} from "react";
import {Message, useWebSocketContext} from "@/app/lib/web-socket-context";
import {RelativeLidarPoint, ObstacleType, GlobalLidarPoint} from "@/app/types/map-objects";
import {Vector3} from "three";
import {RelativeLidarOrigin, RobotContextType, RobotPosition} from "@/app/types/robot";
import {lidarRelativeToRelative3D} from "@/app/utils/lidar-calculations";


const RobotContext = createContext<RobotContextType>({
    leftWheelSpeed: 0,
    rightWheelSpeed: 0,
    setLeftWheelSpeed: () => { },
    setRightWheelSpeed: () => { },
    excavatorPosition: 0,
    setExcavatorPosition: () => { },
    lidarPoints: [],
    setLidarPoints: () => {},
    lidarOrigin: {
        yawOffset: 0,
        pitch: Math.PI / 4,
        relPos: new Vector3(-0.5, 0.5, 0.75),
    },
    obstacles: [],
    setObstacles: () => { },
    robot: {
        x: 0,
        y: 0,
        rotation: 0,
        posConfidenceRect: {
            x1: 0,
            y1: 0,
            x2: 0,
            y2: 0,
        }
    },
    setRobot: () => { },
});

export const useRobotContext = () => useContext(RobotContext);

export default function RobotContextProvider({ children }: { children: ReactNode; }) {

    const [leftWheelSpeed, setLeftWheelSpeed] = useState<number>(0);
    const [rightWheelSpeed, setRightWheelSpeed] = useState<number>(0);
    const [excavatorPosition, setExcavatorPosition] = useState<number>(0);
    const [lidarData, setLidarData] = useState<GlobalLidarPoint[][]>([]);
    const [obstacles, setObstacles] = useState<ObstacleType[]>([]);
    const [robot, setRobot] = useState<RobotPosition>({
        x: 0,
        y: 0,
        rotation: 0,
        posConfidenceRect: {
            x1: 0,
            y1: 0,
            x2: 0,
            y2: 0,
        }
    });

    const lidarOrigin: RelativeLidarOrigin = {
        yawOffset: -Math.PI / 2,
        pitch: -Math.PI / 4,
        relPos: new Vector3(1.35, 1.1, -1.8),
    }

    const { messages } = useWebSocketContext();

    const loadMockData = async () => {
        // const data: Point[] = [];
        // for (let i = 0; i < 300; i++) {
        //     data.push({
        //         distance: Math.random() * 8 + 2,
        //         angle: Math.random() / 2 * Math.PI,
        //         weight: Math.random() * 255,
        //     });
        // }
        // return data;

        // Get the last test data from `Software/ROS2/src/navigation/navigation/obstacles/data.txt`
        //                             `Software/ROS2/src/website/app/lib/robot-context.tsx
        // const pathToData = "../../../../navigation/navigation/obstacles/data.txt";
        // Read the file and parse the data
        fetch('/mock-data/data.txt')
            .then(res => res.text())
            .then(text => {
                const lines = text.trim().split("\n");
                const relPoints: RelativeLidarPoint[] = lines.map(line => {
                    const [x, y] = line.split(",").map(Number);
                    return {
                        distance: Math.sqrt(x * x + y * y)/2.5/6/5, // scale down the distance from error in the data
                        angle: Math.atan2(y, x),
                        weight: 255,
                    };
                });
                const globPoints: GlobalLidarPoint[] = relPoints.map((point) => {
                    const relPos = lidarRelativeToRelative3D(
                        point.distance,
                        point.angle,
                        lidarOrigin,
                    )

                    return {
                        globPos: relPos.add(new Vector3(robot.x, 0, robot.y)),
                        weight: point.weight,
                    };
                });

                setLidarData(prevState => [...prevState, globPoints]);
            })
            .catch(err => {
                console.error("Error reading data.txt", err);
                setLidarData([])
            });
    }

    useEffect(() => {
        if (messages.length == 0) {
            // in the case of a reset, clear the path data
            setLeftWheelSpeed(0);
            setRightWheelSpeed(0);
            setExcavatorPosition(0);
            setLidarData([]);
            // void loadMockData();
            setObstacles([]);
            return;
        }
        const lastMessage = messages[messages.length - 1];
        if (lastMessage.type === "motors") {
            const data = (lastMessage?.message || []);

            setLeftWheelSpeed(data.leftWheelSpeed);
            setRightWheelSpeed(data.rightWheelSpeed);
        } else if (lastMessage.type === "excavator") {
            const data = lastMessage?.message || {};
            setExcavatorPosition(data.armPosition);
        } else if (lastMessage.type === "lidar") {
            const relPoints: RelativeLidarPoint[] = lastMessage?.message || [];

            const globPoints = relPoints.map((point) => {
                const relPos = lidarRelativeToRelative3D(
                    point.distance,
                    point.angle,
                    lidarOrigin,
                )

                return {
                    globPos: relPos.add(new Vector3(robot.x, 0, robot.y)),
                    weight: point.weight,
                };
            });

            setLidarData(prevState => [...prevState, globPoints]);
        } else if (lastMessage.type === "position") {
            const data = lastMessage?.message || {};
            setRobot(prev => ({ ...prev, x: data.x, y: data.y }));
        } else if (lastMessage.type === "orientation") {
            const data = lastMessage?.message || {};
            setRobot(prev => ({ ...prev, rotation: 360.0 - data }));
        } else if (lastMessage.type === "position_confidence") {
            const data = lastMessage?.message || {};
            setRobot(prev => ({
                ...prev,
                posConfidenceRect: {
                    x1: data.x1,
                    y1: data.y1,
                    x2: data.x2,
                    y2: data.y2,
                }
            }));
        }

        const obstaclesMessages = [...new Set(messages
            .filter((message: Message) => message.type === "obstacles")
            .map((message: Message) => message.message)
            .flat()
            .map((newObstacle: any) => {
                return { x: newObstacle.position.x, y: newObstacle.position.y, radius: newObstacle.radius, isHole: !newObstacle.is_rock, relativeX: newObstacle.relative_position.x, relativeY: newObstacle.relative_position.y };
            }))];

        setObstacles(obstaclesMessages);
    }, [messages]);

    return (
        <RobotContext.Provider value={{
            leftWheelSpeed,
            setLeftWheelSpeed,
            rightWheelSpeed,
            setRightWheelSpeed,
            excavatorPosition,
            setExcavatorPosition,
            lidarPoints: lidarData,
            setLidarPoints: setLidarData,
            lidarOrigin,
            obstacles,
            setObstacles,
            robot,
            setRobot,
        }}>
            {children}
        </RobotContext.Provider>
    );
}

