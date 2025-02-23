export default function Map() {
    return (
        <div
            className="flex flex-wrap relative h-full"
            style={{ aspectRatio: "6.88/5", padding: "5vh 0" }}
        >
            <div
                className="relative h-full border-4 border-dashed border-slate-800 border-r-0"
                style={{ aspectRatio: "3.88/5" }}
            >
                <p className="text-center text-sm">Obstacles</p>
                <div className="absolute bg-slate-300 top-1/2 right-0 transform -translate-y-1/2 h-[13%] aspect-square border-4 border-dashed border-slate-800 flex flex-col justify-center items-center">
                    <p className="text-center text-sm">Column</p>
                </div>
                <div className="absolute bottom-0 h-2/5 aspect-square border-4 border-l-0 border-b-0 border-dashed border-slate-800 flex flex-col justify-center items-center">
                    <p className="text-center text-sm">Start</p>
                </div>
            </div>
            <div>
                <div className="aspect-square h-3/5 border-4 border-dashed border-slate-800">
                    <p className="text-center text-sm">Excavation</p>
                </div>
                <div className="aspect-[3/2] h-2/5 border-4 border-dashed border-slate-800 border-t-0">
                    <p className="text-center text-sm">Construction</p>
                </div>
            </div>
        </div>
    );
}
