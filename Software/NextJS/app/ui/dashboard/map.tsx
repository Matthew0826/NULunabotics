export default function Map() {
    return (
        <div
            className="flex flex-wrap h-[98%] m-auto"
            style={{ padding: "1vw", aspectRatio: "6.88/5" }}
        >
            <div
                className="relative h-full border-4 border-dashed border-slate-800 border-r-0"
                style={{ width: "56.395%" }}
            >
                <p className="text-center text-sm">Obstacles</p>
                <div className="absolute bg-slate-300 top-1/2 right-0 transform -translate-y-1/2 aspect-square h-[13%] border-4 border-dashed border-slate-800 flex flex-col justify-center items-center">
                    <p className="text-center text-sm">Column</p>
                </div>
                <div className="absolute bottom-0 h-2/5 aspect-square border-4 border-l-0 border-b-0 border-dashed border-slate-800 flex flex-col justify-center items-center">
                    <p className="text-center text-sm">Start</p>
                </div>
            </div>
            <div className="" style={{ width: "43.605%" }}>
                <div className="h-3/5 border-4 border-dashed border-slate-800">
                    <p className="text-center text-sm">Excavation</p>
                </div>
                <div className="h-2/5 border-4 border-dashed border-slate-800 border-t-0">
                    <p className="text-center text-sm">Construction</p>
                </div>
            </div>
        </div>
    );
}
