"use client";

import {
    useEffect,
    useState,
} from "react";
import Graph from "./components/graph";
import { Dataset } from "@/app/lib/utils";
import { ROSSocketMessage } from "@/app/types/sockets";
import { useWebSocketContext } from "@/app/contexts/web-socket-context";

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
    const { batteryMessages } = useWebSocketContext();

    const [graph, setGraph] = useState<GraphInfo>(graphInfo);
    const [timeCounter, setTimeCounter] = useState(0);

    const [batteryPercent, setBatteryPercent] = useState(0);

    useEffect(() => {

        const currents = batteryMessages.map((message) => message.message.current);
        const voltages = batteryMessages.map((message) => message.message.voltage);
        const percent = batteryMessages.length !== 0 ? batteryMessages[batteryMessages.length - 1].message.percentage : 0;

        setGraph((prevGraph) => ({
            ...prevGraph,
            dataSets: {
                ...prevGraph.dataSets,
                current: {
                    ...prevGraph.dataSets.current,
                    data: currents,
                },
                voltage: {
                    ...prevGraph.dataSets.voltage,
                    data: voltages,
                },
            },
        }));

        setBatteryPercent(percent);
    }, [batteryMessages]);

    return (
        <>
            <div className="w-full overflow-hidden">
                <Graph
                    dataSets={graph.dataSets}
                    title={graph.name}
                    xAxisLabel={graph.xAxisLabel}
                    yAxisLabel={graph.yAxisLabel}
                    timeCounter={timeCounter}
                />
            </div>

            <div className="fixed top-4 right-4 flex items-center bg-white p-2 rounded-lg shadow-lg">
                <div className="w-14 h-6 border-2 border-gray-700 rounded flex items-center relative">

                    <div
                        className={`h-full transition-all duration-300 ${batteryPercent * 100 > 20 ? "bg-green-500" : "bg-red-500"
                            }`}
                        style={{ width: `${Math.floor(batteryPercent * 100)}%` }}
                    />
                    <div className="absolute right-[-6px] w-1.5 h-3 bg-gray-700" />
                </div>

                <span className="ml-2 text-sm font-semibold text-gray-700">
                    {`${Math.floor(batteryPercent * 100)}%`}
                </span>
            </div>
        </>
    );
}
