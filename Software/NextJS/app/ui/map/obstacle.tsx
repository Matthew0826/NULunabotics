/**
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
            className={`absolute bg-slate-400 rounded-full transform -translate-y-1/2 -translate-x-1/2 ${
                isHole
                    ? "shadow-[inset_4px_4px_3px_3px_rgba(0,0,0,0.25)] bg-slate-800"
                    : "drop-shadow-[3px_3px_rgba(0,0,0,0.25)]"
            }`}
            style={{
                width: `${(100 * (2 * radius)) / 6.88}%`,
                height: `${(100 * (2 * radius)) / 5}%`,
                bottom: `${(100 * y) / 5 - 12.5}%`,
                left: `${(100 * x) / 6.88}%`,
            }}
        />
    );
}
