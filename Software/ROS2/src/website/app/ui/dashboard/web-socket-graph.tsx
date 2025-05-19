"use client";

import {
    use,
    useCallback,
    useContext,
    useEffect,
    useRef,
    useState,
} from "react";
import Graph from "./graph";
import { Dataset } from "@/app/lib/utils";
import { Message, useWebSocketContext } from "@/app/lib/web-socket-context";
import BatteryIndicator from "./battery-indicator";

export type GraphInfo = {
    name: string;
    xAxisLabel: string;
    yAxisLabel: string;
    dataSets: {
        [key: string]: Dataset;
    };
};

export default function WebSocketGraph({
    graphInfo,
}: {
    graphInfo: GraphInfo;
}) {
    const { messages, sendToServer } = useWebSocketContext();

    const [graph, setGraph] = useState<GraphInfo>(graphInfo);
    const [timeCounter, setTimeCounter] = useState(0);

    const [batteryPercent, setBatteryPercent] = useState(0);

    useEffect(() => {
        if (messages.length == 0) return;
        const powerMessages = [...new Set(messages
            .filter((message: Message) => message.type === "battery")
            .map((message: Message) => message.message)
            .flat())];
        const currents = powerMessages.map((message: any) => message.current);
        const voltages = powerMessages.map((message: any) => message.voltage);
        const percent = powerMessages.length != 0 ? powerMessages[powerMessages.length - 1].percent : 0;

        const newGraph = { ...graph };
        newGraph.dataSets["current"].data = currents;
        newGraph.dataSets["voltage"].data = voltages;
        setGraph(newGraph);
        setBatteryPercent(percent);
        setTimeCounter((prev) => prev + 1);
    }, [messages]);

    return (
        <>
            <Graph
                key={graph.name}
                dataSets={graph.dataSets}
                title={graph.name}
                xAxisLabel={graph.xAxisLabel}
                yAxisLabel={graph.yAxisLabel}
                timeCounter={timeCounter}
            />

            <div className="fixed top-4 right-4 flex items-center bg-white p-2 rounded-lg shadow-lg">
                <div className="w-14 h-6 border-2 border-gray-700 rounded flex items-center relative">

                    <div
                        className={`h-full transition-all duration-300 ${batteryPercent * 100 > 20 ? "bg-green-500" : "bg-red-500"
                            }`}
                        style={{ width: `${Math.floor(batteryPercent * 100)}%` }}
                    />
                    <div className="absolute right-[-6px] w-1.5 h-3 bg-gray-700" />
                </div>
            </div>
        </>
    );
}
