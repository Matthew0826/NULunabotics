"use client";

import { tempGraphPower } from "@/app/lib/temp-graph-info";
import Map from "../map/map";
import Panel from "./panel";
import WebSocketGraph from "./web-socket-graph";
import BatteryIndicator from "./battery-indicator";
import Timer from "./timer";
import LidarVisual from "./lidar-visual";
import ObjModelAnimator from "../three/obj";
import SpeedSlider from "./speed-slider";
import { useState } from "react";
import { useKeyboardController } from "@/app/lib/keyboard-controller";
import { useGamepadManagerContext } from "./gamepad-state-provider";
import SerialPortDisplay from "./serial-port-display";
import ConfigPanel from "./config-panel";
import ExcavatorVisual from "./excavator_temp_visual";

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
                <Panel title="LiDAR">
                    <LidarVisual />
                </Panel>
                <Panel title="Excavator">
                    <ExcavatorVisual />
                </Panel>
                <Panel title="Config">
                    <ConfigPanel />
                </Panel>
                <Panel title="Serial Ports">
                    <SerialPortDisplay />
                </Panel>
                <Panel title="Wheel Speed">
                    <SpeedSlider value={speed} setValue={setSpeed} />
                </Panel>
                {/* <Panel title="">
                    <ObjModelAnimator
                        objUrl="/models/lunabot4.obj"
                        mtlUrl="/models/lunabot4.mtl"
                        height={'50vh'}
                        controlsConfig={{
                            enablePan: false,
                            enableZoom: false,
                            autoRotate: false,
                            // Limit vertical rotation (in radians)
                            maxPolarAngle: Math.PI * 0.75,  // Can't go below model
                            minPolarAngle: Math.PI * 0.25   // Can't go fully above model
                        }} />
                </Panel> */}
                <Panel title="Power">
                    <WebSocketGraph graphInfo={tempGraphPower} />
                </Panel>
            </div>
            <div className="w-[66vw] hidden xl:block flex-2">{mapPanel}</div>
        </div>
    );
}
