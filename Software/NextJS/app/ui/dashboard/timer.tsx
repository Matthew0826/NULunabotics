"use client";
import * as React from "react";

type TimerProps = {
startTime: number; // Starting time in seconds
};

function Timer({ startTime }: TimerProps) {
const [elapsedTime, setElapsedTime] = React.useState<number>(startTime);
const [isRunning, setIsRunning] = React.useState<boolean>(false);
const intervalRef = React.useRef<NodeJS.Timeout | null>(null); // To store the interval ID

// Start the timer
const startTimer = () => {
if (!isRunning) {
setIsRunning(true);
intervalRef.current = setInterval(() => {
setElapsedTime((prevTime) => prevTime + 1);
}, 1000);
}
};

// Stop the timer
const stopTimer = () => {
setIsRunning(false);
if (intervalRef.current) {
clearInterval(intervalRef.current);
}
};

// Reset the timer
const resetTimer = () => {
stopTimer();
};

const min = Math.floor(elapsedTime / 60);
const sec = elapsedTime % 60;

return (
<div>
<div>
<span className="time">{min}</span>
<span className="unit">min</span>
<span className="time right">{sec.toString().padStart(2, "0")}</span>
<span className="unit">sec</span>
</div>
<div>
{!isRunning ? (
<button onClick={startTimer}>Start</button>
) : (
<button onClick={stopTimer}>Stop</button>
)}
<button onClick={resetTimer}>Stop</button>
</div>
</div>
);
}

export default Timer;