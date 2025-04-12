import { MAP_HEIGHT, MAP_WIDTH } from "./map";


export default function Robot({
    x,
    y,
    width,
    height,
    rotation
}: {
    x: number;
    y: number;
    width: number;
    height: number;
    rotation: number;
}) {
    return (
        <div
            className={`absolute rounded-lg -translate-y-1/2 -translate-x-1/2 bg-blue-400 rotate-[${rotation}deg]`}
            style={{
                width: `${(width) / MAP_WIDTH}%`,
                height: `${(height) / MAP_HEIGHT}%`,
                top: `${y / MAP_HEIGHT}%`,
                left: `${x / MAP_WIDTH}%`,
            }}
        />
    );
}
