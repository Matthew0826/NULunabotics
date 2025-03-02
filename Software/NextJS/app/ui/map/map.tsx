import Obstacle from "./obstacle";
import RobotPath from "./robot-path";

export const MAP_WIDTH = 6.88; // meters
export const MAP_OBSTACLES_ZONE_WIDTH = 3.88; // meters
export const MAP_HEIGHT = 5; // meters
export const COLUMN_WIDTH = 0.8; // meters

// Note: all obstacles and paths are in centimeters
export default function Map() {
    return (
        <div style={{ padding: "1vw" }}>
            <div
                className="flex flex-wrap relative mx-auto"
                style={{
                    aspectRatio: `${MAP_WIDTH}/${MAP_HEIGHT}`,
                    maxHeight: "82vh",
                }}
            >
                <div
                    className="relative h-full border-4 border-slate-800 border-r-0"
                    style={{
                        width: `${
                            (100 * MAP_OBSTACLES_ZONE_WIDTH) / MAP_WIDTH
                        }%`,
                    }}
                >
                    <p className="text-center text-sm">Obstacles</p>
                    <div
                        className="absolute bg-slate-300 top-1/2 right-[-4px] transform -translate-y-1/2 border-4 border-slate-800 flex flex-col justify-center items-center"
                        style={{
                            width:
                                Math.floor(
                                    (100 * COLUMN_WIDTH) /
                                        MAP_OBSTACLES_ZONE_WIDTH
                                ) + "%",
                            height: (100 * COLUMN_WIDTH) / MAP_HEIGHT + "%",
                        }}
                    >
                        <p className="text-center text-sm">Column</p>
                    </div>
                    <div className="absolute bottom-0 h-2/5 aspect-square border-4 border-l-0 border-b-0 border-slate-800 flex flex-col justify-center items-center">
                        <p className="text-center text-sm">Start</p>
                    </div>
                </div>
                <div
                    className=""
                    style={{
                        width: `${
                            (100 * (MAP_WIDTH - MAP_OBSTACLES_ZONE_WIDTH)) /
                            MAP_WIDTH
                        }%`,
                    }}
                >
                    <div className="h-3/5 border-4 border-slate-800">
                        <p className="text-center text-sm">Excavation</p>
                    </div>
                    <div className="h-2/5 border-4 border-slate-800 border-t-0">
                        <p className="text-center text-sm">Construction</p>
                    </div>
                </div>
                <Obstacle x={200} y={400} radius={30} isHole={false} />
                <Obstacle x={300} y={100} radius={40} isHole={true} />
                <RobotPath
                    path={[
                        [100, 400],
                        [100, 180],
                        [500, 180],
                        [550, 400],
                    ]}
                />
            </div>
        </div>
    );
}
