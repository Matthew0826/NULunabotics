"use client";

import { tempGraphPower } from "@/app/lib/temp-graph-info";
import Map from "../map/map";
import Panel from "./panel";
import WebSocketGraph from "./web-socket-graph";
import Timer from "./timer";
import LidarVisual from "./lidar-visual";
import Slider from "./slider";
import { useGamepadManagerContext } from "./gamepad-state-provider";
import SerialPortDisplay from "./serial-port-display";
import ConfigPanel from "./config-panel";
import ExcavatorVisual from "./excavator_temp_visual";
import OrientationCorrection from "./orientation-correction";
import ControlsReminder from "./controls-reminder";
import Render3DRobotModel from "@/app/ui/three/render-robot";

export default function DashboardPanels() {
    const mapPanel = (
        <Panel title="Map" canHide={false}>
            <Map />
        </Panel>
    );

    const { state, setState, speed, setSpeed } = useGamepadManagerContext();

    return (
        <div className="flex flex-row gap-4 p-6 h-full w-full">
            <div className="flex flex-col gap-4 w-full flex-1 overflow-y-scroll overflow-x-hidden scrollbar-hidden">
                {/* <div className="xl:hidden">{mapPanel}</div> */}
                <div className="xl:hidden">{mapPanel}</div>
                {/* <BatteryIndicator /> */}
                <Panel title="Autonomous" canHide={false}>
                    <Timer startTime={0} />
                </Panel>
                <Panel title="Excavator">
                    <ExcavatorVisual />
                </Panel>
                <Panel title="Power">
                    <WebSocketGraph graphInfo={tempGraphPower} />
                </Panel>
                <Panel title="Orientation Correction">
                    <OrientationCorrection />
                </Panel>
                <Panel title="Wheel Speed" hiddenByDefault={true}>
                    <Slider labels={[0.0, 0.5, 1.0]} value={speed} setValue={setSpeed} />
                </Panel>
                <Panel title="Config" hiddenByDefault={true}>
                    <ConfigPanel />
                </Panel>
                <Panel title="LiDAR" hiddenByDefault={true}>
                    <LidarVisual />
                </Panel>
                <Panel title="Serial Ports" hiddenByDefault={true}>
                    <SerialPortDisplay />
                </Panel>
                <Panel title="Controls" hiddenByDefault={true}>
                    <ControlsReminder />
                </Panel>
                <Panel title="">
                    <Render3DRobotModel
                        baseFilename="/models/lunabot_base"
                        wheelFilename="/models/lunabot_wheel"
                        excavatorFilename="/models/lunabot_excavator"
                        height={'50vh'}
                        controlsConfig={{
                            enablePan: false,
                            enableZoom: false,
                            autoRotate: false,
                            // Limit vertical rotation (in radians)
                            maxPolarAngle: Math.PI * 0.75,  // Can't go below model
                            minPolarAngle: Math.PI * 0.25   // Can't go fully above model
                        }} />
                </Panel>
            </div>
            <div className="w-[66vw] hidden xl:block flex-2">{mapPanel}</div>
        </div>
    );
}
