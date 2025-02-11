"use client";

import { useCallback, useEffect, useRef, useState } from "react";
import Graph from "./graph";
import { Dataset } from "@/app/lib/utils";

type GraphInfo = {
    name: string;
    xAxisLabel: string;
    yAxisLabel: string;
    dataSets: {
        [key: string]: Dataset;
    };
};

export default function TesterMessages() {
    const socketRef = useRef<WebSocket | null>(null);
    useEffect(() => {
        if (typeof window === "undefined") return;

        const url = `ws://${window.location.host}/socket`;
        console.log(url);
        socketRef.current = new WebSocket(url);
        return () => {
            socketRef.current?.close();
            socketRef.current = null;
        };
    }, []);

    const [messages, setMessages] = useState<string[]>([]);
    const onMessage = useCallback((message: string) => {
        socketRef.current?.send(message);
    }, []);

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

    useEffect(() => {
        async function handleMessage(event: MessageEvent) {
            const payload =
                typeof event.data === "string"
                    ? event.data
                    : await event.data.text();
            setMessages((p) => [...p, payload]);
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
        }

        socketRef.current?.addEventListener("message", handleMessage);
        socketRef.current?.addEventListener("open", () => {
            socketRef.current?.send("A new page opened to control the robot.");
        });
        return () =>
            socketRef.current?.removeEventListener("message", handleMessage);
    }, []);

    useEffect(() => {
        // send a message on key press
        function handleKeyDown(event: KeyboardEvent) {
            socketRef.current?.send(event.key);
            console.log("sending", event.key);
        }
        window.addEventListener("keydown", handleKeyDown);
        return () => window.removeEventListener("keydown", handleKeyDown);
    });

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
