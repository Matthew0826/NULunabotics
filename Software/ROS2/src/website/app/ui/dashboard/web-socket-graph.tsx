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
import { useWebSocketContext } from "@/app/lib/web-socket-context";

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

    useEffect(() => {
        if (messages.length == 0) return;
        const data = messages[messages.length - 1];
        if (!data.type.startsWith("graph")) return;
        for (const dataPoint of data.message) {
            const graphName = dataPoint.graph as string;
            if (graph.name != graphName) {
                continue;
            }
            const dataSetName = dataPoint.dataSet as string;
            const newData = dataPoint.newData as number[];
            const dataSet = graph.dataSets[dataSetName];
            dataSet.data.push(...newData);
            graph.dataSets[dataSetName] = dataSet;
            setGraph({ ...graph });
        }
    }, [messages]);

    return (
        <Graph
            key={graph.name}
            dataSets={graph.dataSets}
            title={graph.name}
            xAxisLabel={graph.xAxisLabel}
            yAxisLabel={graph.yAxisLabel}
        />
    );
}
