import { MAP_HEIGHT, MAP_WIDTH } from "./map";

/**
 * This component represents where the robot thinks an obstacle is on the map.
 * x, y, and radius should all be in meters relative to the bottom left hand corner.
 * Up and right are positive.
 * */
export default function Obstacle({
    x,
    y,
    radius,
    isHole,
}: {
    x: number;
    y: number;
    radius: number;
    isHole: boolean;
}) {
    return (
        <div
            className={`absolute rounded-full -translate-y-1/2 -translate-x-1/2 ${
                isHole
                    ? "shadow-[inset_4px_4px_3px_3px_rgba(0,0,0,0.25)] bg-zinc-800"
                    : "drop-shadow-[3px_3px_rgba(0,0,0,0.25)] bg-slate-600"
            }`}
            style={{
                width: `${(2 * radius) / MAP_WIDTH}%`,
                height: `${(2 * radius) / MAP_HEIGHT}%`,
                bottom: `${y / MAP_HEIGHT - 12.5}%`, // 12.5% is arbitrary, I'm not sure why you have to subtract it
                left: `${x / MAP_WIDTH}%`,
            }}
        />
    );
}
