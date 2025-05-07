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
    );
}
