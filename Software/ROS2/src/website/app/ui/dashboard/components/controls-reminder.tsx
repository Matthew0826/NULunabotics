export default function ControlsReminder() {
    return (
        <div className="flex flex-col items-center justify-center h-full w-full" >
            <p className="text-center" >
                <span className="font-bold">Controls (Keyboard / Controller):</span> <br />
                <span className="font-bold">WASD / Left Joystick</span> - Forward <br />
                <span className="font-bold">Q / Button L</span> - Toggle between opening the bin and lifting the excavator <br />
                <span className="font-bold">E / Button R</span> - Run the conveyor to lift dust <br />
                <span className="font-bold">{"Keys < and > / Right Joystick"}</span> - Move bin / lift excavator <br />
            </p>
        </div>
    )
}