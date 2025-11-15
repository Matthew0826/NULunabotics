"use client";

import { useEffect, useRef } from "react";
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
    const labels = Array.from({ length: 10 }, (_, i) => i - 10);
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
                    beginAtZero: true
                },
            },
        },
    };
    const chartRef = useRef<HTMLCanvasElement>(null);
    const chartInstance = useRef<Chart>(null);

    useEffect(() => {
        if (!chartInstance.current) {
            chartInstance.current = new Chart(chartRef.current!!, config);
        } else {
            chartInstance.current.data = data;
            chartInstance.current.update('none');
        }

        return () => {
            if (chartInstance.current) {
                chartInstance.current.destroy();
                chartInstance.current = null;
            }
        };
    }, [data, config]);

    return <canvas ref={chartRef} />;
}
