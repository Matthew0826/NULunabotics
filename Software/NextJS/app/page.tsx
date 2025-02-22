import DirectionalDisplay from "./ui/dashboard/directional-display";
import GamepadManager from "./ui/dashboard/gamepad-manager";
import Graph from "./ui/dashboard/graph";
import Map from "./ui/dashboard/map";
import Panel from "./ui/dashboard/panel";
import TesterMessages from "./ui/dashboard/tester-messages";

export default function Page() {
    const handleReceiveData = (data: any) => {};
    return (
        <main className="flex min-h-screen flex-col p-6">
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
            <div className="flex flex-row gap-4 w-full">
                <div className="flex flex-col gap-4 grow-1">
                    <Panel title="Timer">
                        <p>Hello, timer goes here!</p>
                    </Panel>
                    <Panel title="Power">
                        <TesterMessages graph="Power" />
                    </Panel>
                    <Panel title="Other Graph">
                        <TesterMessages graph="Other Graph" />
                    </Panel>
                </div>
                <div className="w-full h-max">
                    <Panel title="Map">
                        <Map />
                    </Panel>
                </div>
            </div>
        </main>
    );
}
