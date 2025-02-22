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

const tempGraphPower = {
    name: "Power",
    xAxisLabel: "time (s)",
    yAxisLabel: "watts (W)",
    dataSets: {
        "Test Data": {
            label: "Test Data",
            color: "rgb(0, 0, 0)",
            data: [],
        },
    },
};
const tempGraphOther = {
    name: "Other Graph",
    xAxisLabel: "time (s)",
    yAxisLabel: "Beasts",
    dataSets: {
        Sin: {
            label: "Unicorns",
            color: "rgb(193, 71, 71)",
            data: [],
        },
        Cos: {
            label: "Dragons",
            color: "rgb(93, 71, 193)",
            data: [],
        },
    },
};

export default function TesterMessages({ graph }: { graph: string }) {
    const { messages, sendToServer } = useWebSocketContext();

    const [graphs, setGraphs] = useState<Array<GraphInfo>>(
        graph == "Power" ? [tempGraphPower] : [tempGraphOther]
    );

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
        if (!graph) return;
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

    return (
        <>
            <div style={{ width: 375 }}>
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

            <details>
                <summary>Incoming Message Logs</summary>
                <div style={{ maxWidth: "50vh" }}>
                    {messages.map((message, index) => (
                        <p key={index}>{message}</p>
                    ))}
                </div>
            </details>
        </>
    );
}
