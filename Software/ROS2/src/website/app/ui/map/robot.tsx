import { MAP_HEIGHT, MAP_WIDTH } from "./map";
import {Rect} from "@/app/types/robot";

export default function Robot({
    x,
    y,
    width,
    height,
    rotation,
    confidenceRect
}: {
    x: number;
    y: number;
    width: number;
    height: number;
    rotation: number;
    confidenceRect: Rect;
}) {
    return (
        <>
            <div
                className={`absolute rounded-lg bg-blue-400`}
                style={{
                    width: `${(width) / MAP_WIDTH}%`,
                    height: `${(height) / MAP_HEIGHT}%`,
                    top: `${y / MAP_HEIGHT}%`,
                    left: `${x / MAP_WIDTH}%`,
                    transform: `translate(-50%, -50%) rotate(${rotation + 90}deg)`,
                }}
            >
                <div className="absolute rounded-lg bg-slate-800"
                    style={{
                        width: `50%`,
                        height: `40%`,
                        top: `20%`,
                        left: `50%`,
                        transform: `translate(-50%, -50%)`,
                    }} />
                <div className="absolute rounded-full bg-white"
                    style={{
                        width: `10%`,
                        height: `7.5%`,
                        top: `50%`,
                        left: `50%`,
                        transform: `translate(-50%, -50%)`,
                    }} />
            </div>
            {confidenceRect && (confidenceRect.x1 != 0 || confidenceRect.y1 != 0) && (
                <div
                    className={`absolute border-2 border-red-500`}
                    style={{
                        width: `${(confidenceRect.x2 - confidenceRect.x1) / MAP_WIDTH}%`,
                        height: `${(confidenceRect.y2 - confidenceRect.y1) / MAP_HEIGHT}%`,
                        top: `${(confidenceRect.y1 + (confidenceRect.y2 - confidenceRect.y1) / 2) / MAP_HEIGHT}%`,
                        left: `${(confidenceRect.x1 + (confidenceRect.x2 - confidenceRect.x1) / 2) / MAP_WIDTH}%`,
                        transform: `translate(-50%, -50%)`,

                    }}
                />)}
        </>
    );
}
