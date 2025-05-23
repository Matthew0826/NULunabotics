"use client";

import { createContext, ReactNode, useContext, useEffect, useState } from "react";
import { RelativeLidarPoint, ObstacleType, GlobalLidarPoint } from "@/app/types/map-objects";
import { Vector3 } from "three";
import { RelativeLidarOrigin, RobotContextType, RobotPosition } from "@/app/types/robot";
import { lidarRelativeToRelative3D } from "@/app/utils/lidar-calculations";
import { ObstacleROSMessage, ROSSocketMessage } from "@/app/types/sockets";
import { useWebSocketContext } from "@/app/contexts/web-socket-context";


const RobotContext = createContext<RobotContextType>({
    leftWheelSpeed: 0,
    rightWheelSpeed: 0,
    setLeftWheelSpeed: () => { },
    setRightWheelSpeed: () => { },
    excavatorPosition: 0,
    setExcavatorPosition: () => { },
    lidarRelativePoints: [],
    setLidarRelativePoints: () => { },
    lidarPoints: [],
    setLidarPoints: () => { },
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
        width: 0,
        height: 0,
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
    const [lidarRelativePoints, setLidarRelativePoints] = useState<RelativeLidarPoint[]>([]);
    const [obstacles, setObstacles] = useState<ObstacleType[]>([]);
    const [robot, setRobot] = useState<RobotPosition>({
        x: 448,
        y: 100,
        width: 68,
        height: 98,
        rotation: 0,
        posConfidenceRect: {
            x1: 0,
            y1: 0,
            x2: 0,
            y2: 0,
        }
    });

    const lidarOrigin: RelativeLidarOrigin = {
        // yawOffset: 0,
        yawOffset: -Math.PI / 2,
        pitch: -Math.PI / 4,
        // relPos: new Vector3(19, 57, 37), //irl cm
        relPos: new Vector3(-0.95, 0.82, -1.8),
    }

    const { latestMessages, obstacleMessages } = useWebSocketContext();

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
                        distance: Math.sqrt(x * x + y * y) / 2.5 / 6 / 5, // scale down the distance from error in the data
                        angle: Math.atan2(y, x),
                        weight: 255,
                    };
                });
                const globPoints: GlobalLidarPoint[] = relPoints.map((point) => {
                    const relPos = lidarRelativeToRelative3D(
                        point.distance,
                        point.angle,
                        lidarOrigin,
                        robot.rotation,
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

    // Called once on mount
    useEffect(() => {
        setLeftWheelSpeed(0);
        setRightWheelSpeed(0);
        setExcavatorPosition(0);
        setLidarData([]);
        // void loadMockData();
        setObstacles([]);
    }, []);

    useEffect(() => {
        for (const message of latestMessages) {

            if (message.type === "motors") {
                const data = (message?.message || []);
                setLeftWheelSpeed(data.front_left_wheel);
                setRightWheelSpeed(data.front_right_wheel);
            } else if (message.type === "excavator_percent") {
                const data = message?.message || {};
                setExcavatorPosition(data.excavator_lifter_percent);
            } else if (message.type === "lidar") {
                const relPoints: RelativeLidarPoint[] = message?.message || [];

                setLidarRelativePoints(relPoints);
                const globPoints = relPoints.map((point) => {
                    const relPos = lidarRelativeToRelative3D(
                        point.distance / 100,
                        point.angle,
                        lidarOrigin,
                        robot.rotation,
                    )

                    return {
                        globPos: relPos.add(new Vector3(robot.x, 0, robot.y)),
                        weight: point.weight,
                    };
                });

                // console.log("globPoints", globPoints);
                // Only take every 6th point so we don't overload the system
                const everyThrid = globPoints.filter((_, index) => index % 20 === 0);
                setLidarData(prevState => [...prevState, globPoints].splice(-1));
            } else if (message.type === "position") {
                const data = message?.message || {};
                setRobot(prev => ({ ...prev, x: data.x, y: data.y }));
            } else if (message.type === "orientation") {
                const data = message?.message || 0;
                setRobot(prev => ({ ...prev, rotation: data + 90 }));
            } else if (message.type === "position_confidence") {
                const data = message?.message || {};
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
        }
    }, [latestMessages]);

    useEffect(() => {
        const obstaclesMessages = obstacleMessages
            .map((newObstacle: ObstacleROSMessage) => {
                return { x: newObstacle.message.position.x, y: newObstacle.message.position.y, radius: newObstacle.message.radius, isHole: !newObstacle.message.is_rock, relativeX: newObstacle.message.relative_position.x, relativeY: newObstacle.message.relative_position.y };
            });

        setObstacles(obstaclesMessages);
    }, [obstacleMessages]);

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
            lidarRelativePoints,
            setLidarRelativePoints,
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

