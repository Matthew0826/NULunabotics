"use client";

import { tempGraphPower } from "@/app/lib/temp-graph-info";
import Map from "../map/map";
import Panel from "./panels/panel";
import WebSocketGraph from "./web-socket-graph";
import Timer from "./components/timer";
import SerialPortPanel from "./panels/serial-port-panel";
import ExcavatorVisual from "./panels/excavator_visual_panel";
import OrientationCorrectionPanel from "./panels/orientation-correction-panel";
import ControlsPanel from "./panels/controls-panel";
import Render3DRobotModel from "@/app/ui/three/render-robot";
import WheelSpeedPanel from "./panels/wheel-speed";
import ConfigPanel from "./panels/config-panel";

export default function DashboardPanels() {
    const mapPanel = (
        <Panel title="Map" canHide={false}>
            <Map />
        </Panel>
    );

    return (
        <div className="flex flex-row gap-2 p-2 h-full w-full">
            <div className="flex flex-col gap-2 w-full flex-1 overflow-y-auto overflow-x-hidden scrollbar-hidden max-h-screen">
                {/* <div className="xl:hidden">{mapPanel}</div> */}
                <div className="xl:hidden">{mapPanel}</div>
                <Panel title={'Robot 3D View'} canHide={false} contentPadding={false}>
                    <Render3DRobotModel
                        baseFilename="/models/theia/chasis"
                        wheelFilename="/models/theia/centerWheel"
                        excavatorFilename="/models/theia/excavator"
                        jointLFilename="/models/theia/jointL"
                        jointRFilename="/models/theia/jointR"
                        motorsFilename="/models/theia/scuffedMotors"
                        height={'50vh'}
                        controlsConfig={{
                            enablePan: false,
                            enableZoom: false,
                            autoRotate: false,
                            // Limit vertical rotation (in radians)
                            maxPolarAngle: Math.PI * 0.502,  // Can't go below model
                            minPolarAngle: Math.PI * 0.25   // Can't go fully above model
                        }} />
                </Panel>
                {/*<BatteryIndicator />*/}
                <Panel title="Autonomous" canHide={false}>
                    <Timer startTime={0} />
                </Panel>
                <Panel title="Excavator">
                    <ExcavatorVisual />
                </Panel>
                <Panel title="Power">
                    <WebSocketGraph graphInfo={tempGraphPower} />
                </Panel>
                <Panel title="Orientation Correction" hiddenByDefault={true}>
                    <OrientationCorrectionPanel />
                </Panel>
                <Panel title="Wheel Speed" hiddenByDefault={true}>
                    <WheelSpeedPanel />
                </Panel>
                <Panel title="Config" hiddenByDefault={false}>
                    <ConfigPanel />
                </Panel>
                <Panel title="Serial Ports" hiddenByDefault={true}>
                    <SerialPortPanel />
                </Panel>
                <Panel title="Controls" hiddenByDefault={true}>
                    <ControlsPanel />
                </Panel>
            </div>
            <div className="w-[66vw] hidden xl:block flex-2">{mapPanel}</div>
        </div>
    );
}
