import DirectionalDisplay from "./ui/dashboard/directional-display";
import GamepadManager from "./ui/dashboard/gamepad-manager";
import Graph from "./ui/dashboard/graph";
import Map from "./ui/dashboard/map";
import Panel from "./ui/dashboard/panel";
import TesterMessages from "./ui/dashboard/tester-messages";

// const url = "ws://localhost:3000/socket";
// const socket = new WebSocket(url);
// console.log("hi");

// socket.addEventListener("open", () => {
//     console.log("WebSocket is open now.");
//     sendMessageEvery3Seconds(socket);
// });

// socket.addEventListener("message", (event) => {
//     console.log(event.data);
// });

export default function Page() {
    const handleReceiveData = (data: any) => {};
    return (
        <main className="flex min-h-screen flex-col p-6">
            <div className="mt-4 flex grow flex-col gap-4 md:flex-row">
                <div className="flex flex-col justify-center gap-6 rounded-lg bg-gray-50 px-6 py-10 md:w-2/5 md:px-20">
                    {/* <p
                        className={`text-xl text-gray-800 md:text-3xl md:leading-normal`}
                    >
                        <strong>NU Lunabotics Dashboard</strong>
                        <br />
                        Game controller tester:
                    </p> */}
                    <GamepadManager />
                    {/* <DirectionalDisplay />
                    <DirectionalDisplay /> */}
                    <div className="flex flex-row gap-4">
                        <div className="flex flex-col gap-4">
                            <Panel>
                                <h1>Graph1</h1>
                                <TesterMessages />
                            </Panel>
                        </div>
                        <Panel>
                            <Map />
                        </Panel>
                    </div>
                </div>
            </div>
        </main>
    );
}
