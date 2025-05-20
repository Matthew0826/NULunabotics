import {createContext, Dispatch, ReactNode, SetStateAction, useContext, useEffect, useRef, useState} from "react";
import {tempStartingData} from "@/app/lib/temp-graph-info";
import {Message, useWebSocketContext} from "@/app/lib/web-socket-context";
import {Point} from "@/app/lib/ros2";
import {ObstacleType} from "@/app/ui/map/map";

type RobotContextType = {
    leftWheelSpeed: number;
    rightWheelSpeed: number;
    setLeftWheelSpeed: Dispatch<SetStateAction<number>>;
    setRightWheelSpeed: Dispatch<SetStateAction<number>>;
    excavatorPosition: number;
    setExcavatorPosition: Dispatch<SetStateAction<number>>;
};

const RobotContext = createContext<RobotContextType>({
    leftWheelSpeed: 0,
    rightWheelSpeed: 0,
    setLeftWheelSpeed: () => {},
    setRightWheelSpeed: () => {},
    excavatorPosition: 0,
    setExcavatorPosition: () => {},
});

export const useRobotContext = () => useContext(RobotContext);

export default function RobotContextProvider({children}: { children: ReactNode; }) {

    const [leftWheelSpeed, setLeftWheelSpeed] = useState<number>(0);
    const [rightWheelSpeed, setRightWheelSpeed] = useState<number>(0);
    const [excavatorPosition, setExcavatorPosition] = useState<number>(0);
    const [lidarData, setLidarData] = useState<Point[]>([]);
    const [obstacles, setObstacles] = useState<ObstacleType[]>([]);

    const { messages } = useWebSocketContext();

    useEffect(() => {
        if (messages.length == 0) {
            // in the case of a reset, clear the path data
            setLeftWheelSpeed(0);
            setRightWheelSpeed(0);
            setExcavatorPosition(0);
            setLidarData([]);
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
            setExcavatorPosition
        }}>
            {children}
        </RobotContext.Provider>
    );
}

