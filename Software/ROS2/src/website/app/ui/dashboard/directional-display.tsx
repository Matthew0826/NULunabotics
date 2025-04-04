export default function DirectionalDisplay({
    x,
    y,
    buttonPressed,
}: {
    x: number;
    y: number;
    buttonPressed: boolean;
}) {
    return (
        <svg
            width="200"
            height="200"
            viewBox="0 0 200 200"
            fill="none"
            xmlns="http://www.w3.org/2000/svg"
        >
            <g clipPath="url(#clip0_604_2)">
                <circle
                    cx="100"
                    cy="100"
                    r="98"
                    stroke={buttonPressed ? "blue" : "black"}
                    strokeWidth="4"
                />
                <line
                    x1="0"
                    y1="98"
                    x2="200"
                    y2="100"
                    stroke="black"
                    strokeWidth="2"
                />
                <line
                    x1="100"
                    y1="200"
                    x2="100"
                    y2="0"
                    stroke="black"
                    strokeWidth="2"
                />
                <circle cx={`${x}`} cy={`${y}`} r="3.5" fill="black" />

                <path
                    d={`M100 99L${x} ${y}`}
                    stroke="white"
                    strokeLinecap="round"
                />
            </g>
            <defs>
                <clipPath id="clip0_604_2">
                    <rect width="200" height="200" fill="white" />
                </clipPath>
            </defs>
        </svg>
    );
}
