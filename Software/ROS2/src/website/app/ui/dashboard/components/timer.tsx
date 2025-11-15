"use client";
import * as React from "react";
import {useWebSocketContext} from "@/app/contexts/web-socket-context";

type TimerProps = {
    startTime: number; // Starting time in seconds
};

function Timer({ startTime }: TimerProps) {
    const [elapsedTime, setElapsedTime] = React.useState<number>(startTime);
    const [isRunning, setIsRunning] = React.useState<boolean>(false);
    const intervalRef = React.useRef<NodeJS.Timeout | null>(null); // To store the interval ID

    const { sendToServer } = useWebSocketContext();

    // Start the timer
    const startTimer = () => {
        if (!isRunning) {
            setIsRunning(true);
            intervalRef.current = setInterval(() => {
                setElapsedTime((prevTime) => prevTime + 1);
            }, 1000);
            sendToServer("beginAutonomous", {});
        }
    };

    const stopTimer = () => {
        setIsRunning(false);
        if (intervalRef.current) {
            clearInterval(intervalRef.current);
        }
        sendToServer("stopAutonomous", {});
    };

    const resetTimer = () => {
        if (window.confirm("RESET AUTONOMOUS NAVIGATION? THIS IS VERY BAD. YOU PROBABLY DON'T WANT TO DO THIS.")) {
            stopTimer();
            setElapsedTime(0);
            sendToServer("resetAutonomous", {});
        }
    };

    const min = Math.floor(elapsedTime / 60);
    const sec = elapsedTime % 60;
    return (
        <div className="flex-cols items-center content-center justify-center justify-items-center font-mono text-2xl">
            <div>
                <span>{min}</span>
                <span>:</span>
                <span>{sec.toString().padStart(2, "0")}</span>
            </div>
            <div>
                <button
                    className={`p-2 border border-black-800 ${isRunning ? "bg-rose-300" : "bg-emerald-400"} rounded-l-lg`}
                    onClick={isRunning ? stopTimer : startTimer}
                >
                    {isRunning ? "Pause" : "Start"}
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
