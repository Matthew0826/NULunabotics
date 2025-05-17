const getColor = (value: number) => {
    if (value > 0.9 || value < 0.1) {
        return "red-400"
    } else if (value > 0.7 || value < 0.3) {
        return "slate-600"
    } else {
        return "emerald-400"
    }
}

export default function PercentViewer({ description, values }: { description: string, values: number[] }) {
    return (
        <div className="flex flex-col justify-center gap-1">
            <p className="w-full text-center">{description}</p>
            <div className={`h-[20px] bg-slate-800 w-full rounded-full overflow-hidden flex flex-col`}>
                {values.map((value, index) => (
                    <div key={index} className={`h-full bg-${getColor(value)}`} style={{ width: `${value * 100}%` }} />
                ))}
            </div>
            <div className="flex justify-between mt-2">
                <span className="text-xs text-gray-500">0%</span>
                <span className="text-xs text-gray-500">50%</span>
                <span className="text-xs text-gray-500">100%</span>
            </div>
        </div>
    )
}