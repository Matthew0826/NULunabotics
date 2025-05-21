export default function SerialPort({ name, port, topics, handleUploadCode }: { name: string, port: string, topics: string[], handleUploadCode: (port: string) => void }) {
    return (
        <div className="rounded-lg border-2 border-slate-800 flex flex-col items-center justify-center p-3 gap-2">
            <h1 className="text-xl font-bold">{port}</h1>
            <div className="flex flex-col items-center gap-2">
                <div>
                    {topics.map((topic, index) => (
                        <div key={index} className="text-sm">{topic}</div>
                    ))}
                </div>
                <button
                    className="border-2 border-slate-800 px-2 py-1 rounded hover:bg-slate-400 transition-colors duration-200 text-sm"
                    onClick={() => handleUploadCode(port)}
                >Upload {name} Code</button>
            </div>
        </div>
    )
}