"use client";

import { useState } from "react";

// A better slider component with lamba for value handling
export default function Slider({
    labels,
    value,
    setValue,
    min = 0,
    max = 1,
    step = 0.01
}: {
    labels: number[],
    value: number,
    setValue: (val: number) => void,
    min?: number,
    max?: number,
    step?: number
}) {
    const [isDragging, setIsDragging] = useState(false);

    const handleChange = (e: any) => {
        const newValue = parseFloat(e.target.value);
        setValue(newValue);
    };

    // percentage of slider
    const valuePercent = ((value - min) / (max - min)) * 100;

    return (
        <div className="w-full max-w-md mx-auto py-8 px-4">
            <div className="relative">
                <div className="w-full h-1 bg-gray-200 rounded-full">
                    <div
                        className="absolute h-1 bg-zinc-800 rounded-full"
                        style={{ width: `${valuePercent}%` }}
                    />
                </div>
                {labels.map((label, index) => (
                    <div
                        key={index}
                        className="absolute h-3 w-[2px] bg-gray-400 top-1/2 left-1/2 transform -translate-y-1/2"
                        style={{ left: `${index / (labels.length - 1) * 100}%` }}
                    />
                ))}
                <div
                    className={`absolute h-4 w-4 rounded-full bg-zinc-800 shadow transform -translate-y-1/2 -translate-x-1/2 top-1/2 cursor-pointer transition-transform ${isDragging ? 'scale-110' : 'hover:scale-110'}`}
                    style={{ left: `${valuePercent}%` }}
                />
                <input
                    type="range"
                    min={min}
                    max={max}
                    step={step}
                    value={value}
                    onChange={handleChange}
                    onMouseDown={() => setIsDragging(true)}
                    onMouseUp={() => setIsDragging(false)}
                    onTouchStart={() => setIsDragging(true)}
                    onTouchEnd={() => setIsDragging(false)}
                    className="absolute w-full top-0 opacity-0 h-4 cursor-pointer z-10"
                />
            </div>
            <div className="flex justify-between mt-2">
                {labels.map((label, index) => (
                    <span key={index} className="text-xs text-gray-500 font-semibold">
                        {label}
                    </span>
                ))}
            </div>
            <div className="mt-4 flex justify-center">
                <input
                    type="number"
                    min={min}
                    max={max}
                    step={step}
                    value={value}
                    onChange={handleChange}
                    className="border border-gray-300 rounded px-2 py-1 w-24 text-center"
                />
            </div>
        </div>
    );
}