"use client";

import { tempGraphOther, tempGraphPower } from "@/app/lib/temp-graph-info";
import Map from "../map/map";
import Panel from "./panel";
import WebSocketGraph from "./web-socket-graph";

export default function DashboardPanels() {
    const mapPanel = (
        <Panel title="Map">
            <Map />
        </Panel>
    );

    return (
        <div className="flex flex-row gap-4 p-6 h-full w-full">
            <div className="flex flex-col gap-4 w-full flex-1">
                <div className="xl:hidden">{mapPanel}</div>
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
            <div className="w-full hidden xl:block flex-2">{mapPanel}</div>
        </div>
    );
}
