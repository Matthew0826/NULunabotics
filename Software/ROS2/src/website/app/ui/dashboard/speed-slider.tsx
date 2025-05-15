import { useState } from 'react';

export default function SpeedSlider({ value, setValue }: { value: number; setValue: (value: number) => void }) {
    const [isDragging, setIsDragging] = useState(false);

    const handleChange = (e: any) => {
        setValue(parseFloat(e.target.value));
    };

    const percentage = value * 100;

    return (
        <div className="w-full max-w-md mx-auto py-8 px-4">
            <div className="relative">
                <div className="w-full h-1 bg-gray-200 rounded-full">
                    <div
                        className="absolute h-1 bg-zinc-800 rounded-full"
                        style={{ width: `${percentage}%` }}
                    />
                </div>
                <div className="absolute h-3 w-px bg-gray-400 top-1/2 left-1/2 transform -translate-y-1/2" />
                <div
                    className={`absolute h-4 w-4 rounded-full bg-zinc-800 shadow transform -translate-y-1/2 -translate-x-1/2 top-1/2 cursor-pointer transition-transform ${isDragging ? 'scale-110' : 'hover:scale-110'}`}
                    style={{ left: `${percentage}%` }}
                />
                <input
                    type="range"
                    min="0"
                    max="1"
                    step="0.01"
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
                <span className="text-xs text-gray-500">0.0</span>
                <span className="text-xs text-gray-500">0.5</span>
                <span className="text-xs text-gray-500">1.0</span>
            </div>
        </div>
    );
};
