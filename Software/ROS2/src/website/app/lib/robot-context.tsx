"use client";

import {createContext, Dispatch, ReactNode, SetStateAction, useContext, useEffect, useRef, useState} from "react";
import {tempStartingData} from "@/app/lib/temp-graph-info";
import {Message, useWebSocketContext} from "@/app/lib/web-socket-context";
import {Point} from "@/app/lib/ros2";
import {ObstacleType} from "@/app/ui/map/map";

export type Pos3D = {
    x: number; y: number; z: number;
    roll: number; pitch: number; yaw: number;
};

type RobotContextType = {
    leftWheelSpeed: number;
    rightWheelSpeed: number;
    setLeftWheelSpeed: Dispatch<SetStateAction<number>>;
    setRightWheelSpeed: Dispatch<SetStateAction<number>>;
    excavatorPosition: number;
    setExcavatorPosition: Dispatch<SetStateAction<number>>;
    lidarPoints: Point[];
    setLidarPoints: Dispatch<SetStateAction<Point[]>>;
    lidarOrigin: Pos3D;
};

const mockLidarPoints: Point[] = Array.from({ length: 12 }, (_, i) => {
    const angle = (i * 5 * Math.PI) / 180; // convert degrees to radians
    return {
        distance: 4 + 0.2 * Math.sin(i), // some variation in distance
        angle,
        weight: 1.0
    };
});

const RobotContext = createContext<RobotContextType>({
    leftWheelSpeed: 0,
    rightWheelSpeed: 0,
    setLeftWheelSpeed: () => {},
    setRightWheelSpeed: () => {},
    excavatorPosition: 1, // between 0-1
    setExcavatorPosition: () => {},
    lidarPoints: Point,
    setLidarPoints: () => {},
    lidarOrigin: {
        x: -0.5, y: 0.5, z: 0.75,
        roll:0, pitch:Math.PI/4, yaw:Math.PI
    },
});

export const useRobotContext = () => useContext(RobotContext);

export default function RobotContextProvider({children}: { children: ReactNode; }) {

    const [leftWheelSpeed, setLeftWheelSpeed] = useState<number>(0);
    const [rightWheelSpeed, setRightWheelSpeed] = useState<number>(0);
    const [excavatorPosition, setExcavatorPosition] = useState<number>(0);
    const [lidarData, setLidarData] = useState<Point[]>(mockLidarPoints);
    const [obstacles, setObstacles] = useState<ObstacleType[]>([]);

    const lidarOrigin: Pos3D = {
        x: 0.3, y: 0.25, z: -0.5,
        roll:0, pitch:-Math.PI/6, yaw:3*Math.PI/2
    };

    const { messages } = useWebSocketContext();

    useEffect(() => {
        if (messages.length == 0) {
            // in the case of a reset, clear the path data
            setLeftWheelSpeed(0);
            setRightWheelSpeed(0);
            setExcavatorPosition(0.2);
            setLidarData(mockLidarPoints);
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
            const data = lastMessage?.message || [];
            setLidarData(data);
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
            lidarOrigin
        }}>
            {children}
        </RobotContext.Provider>
    );
}

