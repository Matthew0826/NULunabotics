export default function ControlsReminder() {
    return (
        <div className="flex flex-col items-center justify-center h-full w-full" >
            <p className="text-center" >
                <span className="font-bold">Controls:</span> <br />
                <span className="font-bold">W</span> - Forward <br />
                <span className="font-bold">S</span> - Backward <br />
                <span className="font-bold">A</span> - Left <br />
                <span className="font-bold">D</span> - Right <br />
                <span className="font-bold">Q</span> - Toggle between opening the bin and lifting the excavator <br />
                <span className="font-bold">E</span> - Run the conveyor to lift dust <br />
                <span className="font-bold">{"<"}</span> - Close bin / lift excavator <br />
                <span className="font-bold">{">"}</span> - Open bin / lower excavator <br />
            </p>
        </div>
    )
}