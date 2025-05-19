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

    useEffect(() => {
        if (messages.length == 0) return;
        const powerMessages = [...new Set(messages
            .filter((message: Message) => message.type === "power")
            .map((message: Message) => message.message)
            .flat())];
        const currents = powerMessages.map((message: any) => message.current);
        const voltages12v = powerMessages.map((message: any) => message.voltage12v);
        const voltages5v = powerMessages.map((message: any) => message.voltage5v);

        const newGraph = { ...graph };
        newGraph.dataSets["current"].data = currents;
        newGraph.dataSets["12V"].data = voltages12v;
        newGraph.dataSets["5V"].data = voltages5v;
        setGraph(newGraph);
        setTimeCounter((prev) => prev + 1);
    }, [messages]);

    return (
        <Graph
            key={graph.name}
            dataSets={graph.dataSets}
            title={graph.name}
            xAxisLabel={graph.xAxisLabel}
            yAxisLabel={graph.yAxisLabel}
            timeCounter={timeCounter}
        />
    );
}
