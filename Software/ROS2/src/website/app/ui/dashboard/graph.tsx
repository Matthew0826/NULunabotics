"use client";

import { useEffect, useRef, useState } from "react";
import Chart, { ChartConfiguration } from "chart.js/auto";
import { Dataset } from "@/app/lib/utils";

type GraphProps = {
    dataSets: { [id: string]: Dataset };
    title: string;
    xAxisLabel: string;
    yAxisLabel: string;
    timeCounter: number;
};

export default function Graph({
    dataSets,
    title,
    xAxisLabel,
    yAxisLabel,
    timeCounter
}: GraphProps) {
    const dataSetsArray = Object.values(dataSets);
    // these are labels at the bottom of the graph (x axis)
    const labels = Array.from({ length: dataSetsArray[0].data.length }, (_, i) => i + Math.max(timeCounter, 10) - 10);
    const data = {
        labels: labels,
        datasets: dataSetsArray.map((dataSet) => {
            return {
                label: dataSet.label,
                data: dataSet.data,
                borderColor: dataSet.color,
                fill: false,
                cubicInterpolationMode: "monotone",
                tension: 0.4,
            };
        }),
    };

    const config: ChartConfiguration = {
        type: "line",
        data: data,
        options: {
            animation: false,
            responsive: true,
            aspectRatio: 1.7,
            interaction: {
                intersect: false,
            },
            scales: {
                x: {
                    display: true,
                    title: {
                        display: true,
                        text: xAxisLabel,
                    },
                },
                y: {
                    display: true,
                    title: {
                        display: true,
                        text: yAxisLabel,
                    },
                    beginAtZero: true,
                    //   suggestedMin: -10,
                    //   suggestedMax: 200
                },
            },
        },
    };
    const chartRef = useRef(null);

    useEffect(() => {
        const myChart = new Chart(chartRef.current!!, config);

        return () => {
            myChart.destroy();
        };
    }, [data]);

    return <canvas ref={chartRef} />;
}
