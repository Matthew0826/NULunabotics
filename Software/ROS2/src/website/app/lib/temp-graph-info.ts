import {ROSSocketMessage} from "@/app/types/sockets";

export const tempGraphPower = {
    name: "Power",
    xAxisLabel: "time (s)",
    yAxisLabel: "",
    dataSets: {
        "current": {
            label: "Current (A)",
            color: "rgb(95, 188, 123)",
            data: [],
        },
        "voltage": {
            label: "Battery Voltage (V)",
            color: "rgb(14, 18, 37)",
            data: [],
        },
    },
};

const getRock = (i: number) => {
    const obstacle = {
        position: {
            x: 0,
            y: 0,
        },
        radius: 0,
        isHole: false,
    }
    obstacle.position.x = 548.0 - i * 34.25 + 34.25 / 2.0;
    obstacle.position.y = 244.0;
    obstacle.radius = 34.25 / 2.0;
    return obstacle;
}

export const tempStartingData: ROSSocketMessage[] = [
    // {
    //     type: "lidar",
    //     message: [
    //         { distance: 200, angle: (7 * Math.PI) / 4, weight: 0 },
    //         { distance: 100, angle: (6 * Math.PI) / 4, weight: 0 },
    //         { distance: 200, angle: (5 * Math.PI) / 4, weight: 0 },
    //         { distance: 120, angle: Math.PI, weight: 0 },
    //         { distance: 200, angle: (3 * Math.PI) / 4, weight: 0 },
    //         { distance: 200, angle: (2 * Math.PI) / 4, weight: 0 },
    //         { distance: 200, angle: Math.PI / 4, weight: 0 },
    //         { distance: 200, angle: 0, weight: 0 },
    //     ],
    // },
    // {
    //     type: "obstacles",
    //     message: [
    //         getRock(1),
    //         getRock(2),
    //         getRock(3),
    //         getRock(4),
    //         getRock(5),
    //         getRock(6),
    //         getRock(7),
    //         getRock(8),
    //     ]
    // }

]
