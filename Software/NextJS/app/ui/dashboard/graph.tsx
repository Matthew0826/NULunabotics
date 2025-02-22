"use client";

import { useEffect, useRef } from "react";
import Chart, { ChartConfiguration } from "chart.js/auto";
import { Dataset } from "@/app/lib/utils";

type GraphProps = {
    dataSets: { [id: string]: Dataset };
    title: string;
    xAxisLabel: string;
    yAxisLabel: string;
};

export default function Graph({
    dataSets,
    title,
    xAxisLabel,
    yAxisLabel,
}: GraphProps) {
    const dataSetsArray = Object.values(dataSets);
    const labels = dataSetsArray[0].data
        .slice(-10)
        .map((_, i) => i + dataSetsArray[0].data.length);
    const data = {
        labels: labels,
        datasets: dataSetsArray.map((dataSet) => {
            return {
                label: dataSet.label,
                data: dataSet.data.slice(-10),
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
            aspectRatio: 1.5,
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
