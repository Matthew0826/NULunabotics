import {createContext, Dispatch, ReactNode, SetStateAction, useContext, useEffect, useRef, useState} from "react";
import {tempStartingData} from "@/app/lib/temp-graph-info";
import {Message, useWebSocketContext} from "@/app/lib/web-socket-context";

type RobotContextType = {
    leftWheelSpeed: number;
    rightWheelSpeed: number;
    setLeftWheelSpeed: Dispatch<SetStateAction<number>>;
    setRightWheelSpeed: Dispatch<SetStateAction<number>>;
    armPosition: number;
    setArmPosition: Dispatch<SetStateAction<number>>;
};

const RobotContext = createContext<RobotContextType>({
    leftWheelSpeed: 0,
    rightWheelSpeed: 0,
    setLeftWheelSpeed: () => {},
    setRightWheelSpeed: () => {},
    armPosition: 0,
    setArmPosition: () => {},
});

export const useRobotContext = () => useContext(RobotContext);

export default function RobotContextProvider({children}: { children: ReactNode; }) {

    const [leftWheelSpeed, setLeftWheelSpeed] = useState<number>(0);
    const [rightWheelSpeed, setRightWheelSpeed] = useState<number>(0);
    const [armPosition, setArmPosition] = useState<number>(0);

    const { messages } = useWebSocketContext();

    useEffect(() => {
        if (messages.length == 0) {
            // in the case of a reset, clear the path data
            setLeftWheelSpeed(0);
            setRightWheelSpeed(0);
            setArmPosition(0);
            return;
        }
        const lastMessage = messages[messages.length - 1];
        if (lastMessage.type === "motors") {
            const data = (lastMessage?.message || []);

            setLeftWheelSpeed(data.leftWheelSpeed);
            setRightWheelSpeed(data.rightWheelSpeed);
        }
    }, [messages]);

    return (
        <RobotContext.Provider value={{
            leftWheelSpeed,
            setLeftWheelSpeed,
            rightWheelSpeed,
            setRightWheelSpeed,
            armPosition,
            setArmPosition
        }}>
            {children}
        </RobotContext.Provider>
    );
}

