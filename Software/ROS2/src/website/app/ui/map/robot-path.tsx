import React from "react";
import { MAP_HEIGHT, MAP_WIDTH } from "./map";
import {useWebSocketContext} from "@/app/contexts/web-socket-context";
import {MapPoint} from "@/app/types/map-objects";

// This is really badly named sorry
function pointAwayFromPoint(
    point: MapPoint,
    otherPoint: MapPoint,
    distance: number,
    angleModifier: number
): MapPoint {
    const [x, y] = point;
    const [otherX, otherY] = otherPoint;

    const dx = x - otherX;
    const dy = y - otherY;
    const angle = Math.atan2(dy, dx);

    return [
        x - distance * Math.cos(angle + angleModifier),
        y - distance * Math.sin(angle + angleModifier),
    ];
}

/**
 *  This algorithm is really confusing, but it generates a SVG path string that makes a line with corner radius of "radius".
 *  It adds an arrow at the end.
 *  Returns the path string.
 *  Note: This website is really helpful for understanding how SVG paths work:
 *  https://www.nan.fyi/svg-paths/bezier-curves
 *  */
function pointsToPathWithQuadraticCurves(
    path: MapPoint[],
    radius: number,
    drawArrow: boolean = true
): string {
    if (path.length === 0) return "";

    // Start the path string. "M" means "move to"
    let pathString = `M ${path[0][0]} ${path[0][1]}`;
    const duplicatePath = [...path];
    duplicatePath.push(duplicatePath[duplicatePath.length - 1]); // Duplicate last point so it can be used as a control point

    // Loop through all points except the:
    // "move to" point (i=0)
    // and the duplicate point (i=-1)
    for (let i = 1; i < duplicatePath.length - 1; i++) {
        const pointI = duplicatePath[i];

        // Find the point that is a "radius" number units in the direction of point i-1 (away from point i)
        const short = pointAwayFromPoint(
            pointI,
            duplicatePath[i - 1],
            radius,
            0
        );
        // Find the point that is a "radius" units in the direction of point i+1 (away from point i)
        const far = pointAwayFromPoint(pointI, duplicatePath[i + 1], radius, 0);

        // Draw a straight line to the point that is just short of x1 y1
        pathString += ` L ${short[0]} ${short[1]}`;
        if (i == duplicatePath.length - 2 && drawArrow) {
            // At the last point, draw an arrow, accounting for the direction of the path
            const lineToLastPoint = `L ${pointI[0]} ${pointI[1]}`;
            pathString += lineToLastPoint;
            const arrowComponent1 = pointAwayFromPoint(
                duplicatePath[i],
                duplicatePath[i - 1],
                19,
                Math.PI / 4
            );
            pathString += ` L ${arrowComponent1[0]} ${arrowComponent1[1]}`;
            // Mostly removes the dotted line, and goes back to the last point so that it can complete the arrow
            // TODO: make it actually remove the dotted line in the arrow instead of just doubling up the lines
            pathString += lineToLastPoint;
            const arrowComponent2 = pointAwayFromPoint(
                duplicatePath[i],
                duplicatePath[i - 1],
                19,
                -Math.PI / 4
            );
            pathString += ` L ${arrowComponent2[0]} ${arrowComponent2[1]}`;
            // Mostly removes the dotted line
            pathString += lineToLastPoint;
        } else if (i < duplicatePath.length - 2) {
            // Unless its the last point,
            // Draw a curve (Q means quadratic) to the point in the direction of the future point
            // "far" is the end point of the curve, which is the point in the direction of the future point: i+1
            pathString += ` Q ${pointI[0]} ${pointI[1]} ${far[0]} ${far[1]}`;
        }
    }

    return pathString;
}

/**
 * This component represents the path the robot intends to take through the map.
 */
export default function RobotPath({ path, odometryPath, robot }: { path?: MapPoint[], odometryPath: MapPoint[], robot: any }) {
    const { messages, sendToServer } = useWebSocketContext();
    const pathRef = React.useRef<HTMLDivElement>(null);
    const [previousClickPosition, setPreviousClickPosition] = React.useState<MapPoint | null>(null);
    const handlePathClick = (e: React.MouseEvent) => {
        const rect = pathRef.current?.getBoundingClientRect();
        if (!rect) return;
        const x = ((e.clientX - rect.left) / rect.width) * MAP_WIDTH * 100;
        const y = ((e.clientY - rect.top) / rect.height) * MAP_HEIGHT * 100;
        const newClickPosition: MapPoint = [Math.floor(x), Math.floor(y)];
        // check if click is left or right
        if (e.button === 2) { // right click
            // send to server mock obstacle
            sendToServer("mockObstacle", { point: newClickPosition });
        } else if (e.button === 0) { // left click
            sendToServer("sendPathfindingRequest", { point1: previousClickPosition ?? newClickPosition, point2: newClickPosition, robot: robot });
            setPreviousClickPosition(newClickPosition);
        }
    }
    return (
        <div className="absolute top-0 left-0 w-full h-full" ref={pathRef} onClick={(e) => handlePathClick(e)}>
            <svg
                width="100%"
                height="100%"
                viewBox={`0 0 ${MAP_WIDTH * 100} ${MAP_HEIGHT * 100}`}
                fill="none"
                xmlns="http://www.w3.org/2000/svg"
            >
                <path
                    d={pointsToPathWithQuadraticCurves(path ?? [], 10)}
                    stroke="#312B63"
                    strokeWidth="4"
                    strokeLinejoin="round"
                    strokeLinecap="round"
                    strokeDasharray="10 10"
                />
                <path
                    d={pointsToPathWithQuadraticCurves(odometryPath ?? [], 10, false)}
                    stroke="#E85252"
                    strokeWidth="4"
                    strokeLinejoin="round"
                    strokeLinecap="round"
                    strokeDasharray="10 10"
                />
            </svg>
            {/* {
                path?.map((point, index) => (
                    <div
                        key={index}
                        className="absolute rounded-full bg-red-500"
                        style={{
                            width: "10px",
                            height: "10px",
                            left: `${(point[0] / (MAP_WIDTH * 100)) * 100}%`,
                            top: `${(point[1] / (MAP_HEIGHT * 100)) * 100}%`,
                        }}
                    />
                ))
            } */}
        </div>
    );
}
