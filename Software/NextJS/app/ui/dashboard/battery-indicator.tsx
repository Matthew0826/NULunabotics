import { useState, useEffect } from "react";

export default function BatteryIndicator() {
    const [batteryLevel, setBatteryLevel] = useState<number | null>(null);
    const [isSupported, setIsSupported] = useState(true);

    useEffect(() => {
        const updateBattery = async () => {
            if ("getBattery" in navigator) {
                try {
                    const battery = await (navigator as any).getBattery();
                    setBatteryLevel(battery.level * 100);

                    battery.addEventListener("levelchange", () => {
                        setBatteryLevel(battery.level * 100);
                    });
                } catch (error) {
                    console.error("Failed to access battery status:", error);
                    setIsSupported(false);
                }
            } else {
                console.warn(
                    "Battery Status API not supported in this browser."
                );
                setIsSupported(false);
            }
        };

        updateBattery();
    }, []);

    return (
        <div className="fixed top-4 right-4 flex items-center bg-white p-2 rounded-lg shadow-lg">
            <div className="w-14 h-6 border-2 border-gray-700 rounded flex items-center relative">
                {/* Battery Fill (Only if batteryLevel is available) */}
                {batteryLevel !== null ? (
                    <div
                        className={`h-full transition-all duration-300 ${
                            batteryLevel > 20 ? "bg-green-500" : "bg-red-500"
                        }`}
                        style={{ width: `${Math.floor(batteryLevel)}%` }}
                    ></div>
                ) : (
                    <div className="h-full w-full bg-gray-300"></div> // Default empty state
                )}
                {/* Battery Tip */}
                <div className="absolute right-[-6px] w-1.5 h-3 bg-gray-700"></div>
            </div>
            <span className="ml-2 text-sm font-semibold text-gray-700">
                {isSupported
                    ? batteryLevel !== null
                        ? `${batteryLevel}%`
                        : "Loading..."
                    : "N/A"}
            </span>
        </div>
    );
}
