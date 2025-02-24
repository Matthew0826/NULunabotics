import DirectionalDisplay from "./ui/dashboard/directional-display";
import GamepadTester from "./ui/dashboard/gamepad-tester";
import Graph from "./ui/dashboard/graph";
import Map from "./ui/dashboard/map";
import Panel from "./ui/dashboard/panel";
import WebSocketGraph from "./ui/dashboard/web-socket-graph";
import TesterMessages from "./ui/dashboard/web-socket-graph";

const tempGraphPower = {
    name: "Power",
    xAxisLabel: "time (s)",
    yAxisLabel: "watts (W)",
    dataSets: {
        "Test Data": {
            label: "Test Data",
            color: "rgb(0, 0, 0)",
            data: [],
        },
    },
};
const tempGraphOther = {
    name: "Other Graph",
    xAxisLabel: "time (s)",
    yAxisLabel: "Beasts",
    dataSets: {
        Sin: {
            label: "Unicorns",
            color: "rgb(193, 71, 71)",
            data: [],
        },
        Cos: {
            label: "Dragons",
            color: "rgb(93, 71, 193)",
            data: [],
        },
    },
};

export default function Page() {
    const handleReceiveData = (data: any) => {};
    return (
        <main
            className="flex h-screen w-full"
            style={{ justifySelf: "center" }}
        >
            <div className="flex flex-row gap-4 p-6 h-full w-full">
                <div className="flex flex-col gap-4 grow-1">
                    <Panel title="Timer">
                        <p>Hello, timer goes here!</p>
                    </Panel>
                    <Panel title="Power">
                        <WebSocketGraph graphInfo={tempGraphPower} />
                    </Panel>
                    <Panel title="Other Graph">
                        <WebSocketGraph graphInfo={tempGraphOther} />
                    </Panel>
                </div>
                <div className="w-full">
                    <Panel title="Map">
                        <Map />
                    </Panel>
                </div>
            </div>
            {/* <GamepadTester /> */}
        </main>
    );
}
