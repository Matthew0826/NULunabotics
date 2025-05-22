export default function ControlsPanel() {
    const controls = [
        { keyboard: "WASD", controller: "Left Joystick", action: "Move Forward:" },
        { keyboard: "Q", controller: "Button L", action: "Toggle Bin / Excavator Mode:" },
        { keyboard: "E", controller: "Button R", action: "Run Conveyor:" },
        { keyboard: "< and >", controller: "Right Joystick", action: "Move Bin / Lift Excavator:" },
    ];

    return (
        <div className="flex flex-col items-center justify-center h-full w-full p-4">
            <table className="table-auto border border-gray-300">
                <thead>
                <tr className="bg-gray-100">
                    <th className="px-4 py-2 border-b text-left">Action</th>
                    <th className="px-4 py-2 border-b text-left">Keyboard</th>
                    <th className="px-4 py-2 border-b text-left">Controller</th>
                </tr>
                </thead>
                <tbody>
                {controls.map((control, index) => (
                    <tr key={index} className="odd:bg-white even:bg-gray-50">
                        <td className="px-4 py-2 border-b">{control.action}</td>
                        <td className="px-4 py-2 border-b font-semibold">{control.keyboard}</td>
                        <td className="px-4 py-2 border-b font-semibold">{control.controller}</td>
                    </tr>
                ))}
                </tbody>
            </table>
        </div>
    );
}
