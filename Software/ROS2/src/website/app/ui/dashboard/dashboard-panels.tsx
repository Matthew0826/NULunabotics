"use client";

import { tempGraphPower } from "@/app/lib/temp-graph-info";
import Map from "../map/map";
import Panel from "./panel";
import WebSocketGraph from "./web-socket-graph";
import BatteryIndicator from "./battery-indicator";
import Timer from "./timer";
import LidarVisual from "./lidar-visual";

export default function DashboardPanels() {
    const mapPanel = (
        <Panel title="Map">
            <Map />
        </Panel>
    );

    return (
        <div className="flex flex-row gap-4 p-6 h-full w-full">
            <div className="flex flex-col gap-4 w-full flex-1 overflow-y-scroll overflow-x-hidden scrollbar-hidden">
                {/* <div className="xl:hidden">{mapPanel}</div> */}
                <div className="xl:hidden">{mapPanel}</div>
                <BatteryIndicator />

                <Panel title="LiDAR">
                    <LidarVisual />
                </Panel>
                <Panel title="Timer">
                    <Timer startTime={0} />
                </Panel>
                <Panel title="Power">
                    {/* <BatteryIndicator /> */}
                    <WebSocketGraph graphInfo={tempGraphPower} />
                </Panel>
            </div>
            <div className="w-[66vw] hidden xl:block flex-2">{mapPanel}</div>
        </div>
    );
}
