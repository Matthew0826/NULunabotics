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
    const stopTimer = () => {
        setIsRunning(false);
        if (intervalRef.current) {
            clearInterval(intervalRef.current);
        }
    };

    const resetTimer = () => {
        stopTimer();
        setElapsedTime(0);
    };

    const min = Math.floor(elapsedTime / 60);
    const sec = elapsedTime % 60;
    // h-full items-center rounded-xl p-6 gap-x-4 bg-black-900/50 border border-black-800 bg-slate-100
    return (
        <div className="flex-cols items-center content-center justify-center justify-items-center font-mono text-2xl">
            <div>
                <span className="">{min}</span>
                <span className="unit">:</span>
                <span className="">{sec.toString().padStart(2, "0")}</span>
                {/* <span className="unit">sec</span> */}
            </div>
            <div>
                <button
                    className={`p-2 border border-black-800 bg-${isRunning ? "rose" : "emerald"}-400 rounded-l-lg`}
                    onClick={isRunning ? stopTimer : startTimer}
                >
                    {isRunning ? "Stop!" : "Start"}
                </button>
                <button
                    className="p-2 border border-black-800 bg-slate-500 rounded-r-lg"
                    onClick={resetTimer}
                >
                    Reset
                </button>
            </div>
        </div>
    );
}

export default Timer;
