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
import { useWebSocketContext } from "@/app/socket/web-socket-context";

type GraphInfo = {
    name: string;
    xAxisLabel: string;
    yAxisLabel: string;
    dataSets: {
        [key: string]: Dataset;
    };
};

export default function TesterMessages() {
    const { messages, sendToServer } = useWebSocketContext();

    useEffect(() => {
        // send a message on key press
        // temporary!
        function handleKeyDown(event: KeyboardEvent) {
            sendToServer(event.key);
            console.log("sending", event.key);
        }
        window.addEventListener("keydown", handleKeyDown);
        return () => window.removeEventListener("keydown", handleKeyDown);
    });

    useEffect(() => {
        if (messages.length == 0) return;
        const payload = messages[messages.length - 1];
        const data = JSON.parse(payload);
        const graphName = data.graph as string;
        const dataSetName = data.dataSet as string;
        const newData = data.newData as number[];
        const graph = graphs.filter((g) => g.name == graphName)[0];
        const dataSet = graph.dataSets[dataSetName];
        dataSet.data.push(...newData);
        setGraphs((p) =>
            p.map((g) => {
                if (g.name == graphName) {
                    g.dataSets[dataSetName] = dataSet;
                }
                return g;
            })
        );
    }, [messages]);

    const [graphs, setGraphs] = useState<Array<GraphInfo>>([
        {
            name: "Graph 1",
            xAxisLabel: "x-axis",
            yAxisLabel: "y-axis",
            dataSets: {
                "Test Data": {
                    label: "Test Data",
                    color: "rgb(0, 0, 0)",
                    data: [20, 30, 4, 17],
                },
            },
        },
    ]);

    return (
        <>
            <div style={{ width: 600 }}>
                {graphs.map((graph) => (
                    <Graph
                        key={graph.name}
                        dataSets={graph.dataSets}
                        title={graph.name}
                        xAxisLabel={graph.xAxisLabel}
                        yAxisLabel={graph.yAxisLabel}
                    />
                ))}
            </div>

            {/* <details>
                <summary>Incoming Message Logs</summary>
                <div style={{ maxWidth: "50vh" }}>
                    {messages.map((message, index) => (
                        <p key={index}>{message}</p>
                    ))}
                </div>
            </details> */}
        </>
    );
}
