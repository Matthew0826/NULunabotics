export default function Map() {
    return (
        <div
            className="flex flex-wrap bg-black text-white"
            style={{ aspectRatio: "6.88/5" }}
        >
            <div
                className="h-full bg-slate-400"
                style={{ aspectRatio: "3.88/5" }}
            >
                <div
                    className="right-0 margin-auto bg-red-300 aspect-square object-center-right"
                    style={{ height: 20 }}
                />
            </div>
            <div>
                <div className="bg-slate-600 aspect-square h-3/5"></div>
                <div className="bg-slate-800 aspect-[3/2] h-2/5"></div>
            </div>
        </div>
    );
}
