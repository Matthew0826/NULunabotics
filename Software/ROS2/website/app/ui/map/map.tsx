import Obstacle from "./obstacle";
import RobotPath from "./robot-path";

export const MAP_WIDTH = 5.48; // meters
export const MAP_OBSTACLES_ZONE_HEIGHT = 2.44; // meters
export const MAP_HEIGHT = 4.87; // meters
export const COLUMN_WIDTH = 0.8; // meters

// Note: all obstacles and paths are in centimeters
export default function Map() {
    return (
        <div style={{ padding: "1vw" }}>
            <div
                className="flex flex-wrap relative mx-auto gap-0"
                style={{
                    aspectRatio: `${MAP_WIDTH}/${MAP_HEIGHT}`,
                    maxHeight: "82vh",
                }}
            >
                <div
                    className="relative w-full border-4 border-slate-800 border-b-0"
                    style={{
                        height: `${
                            (100 * MAP_OBSTACLES_ZONE_HEIGHT) / MAP_HEIGHT
                        }%`,
                    }}
                >
                    <p className="absolute t-0 left-1/2 transform -translate-x-1/2 text-center text-sm">
                        Obstacles
                    </p>
                    {/* <div
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
                    </div> */}
                    <div
                        className={`absolute right-0 aspect-square border-4 border-r-0 border-t-0 border-slate-800 flex flex-col justify-center items-center`}
                        style={{
                            height: `${Math.floor(
                                (100 * 2) /
                                    (MAP_HEIGHT - MAP_OBSTACLES_ZONE_HEIGHT)
                            )}%`,
                        }}
                    >
                        <p className="text-center text-sm">Start</p>
                    </div>
                </div>
                <div
                    className="flex w-full"
                    style={{
                        height: `${
                            (100 * (MAP_HEIGHT - MAP_OBSTACLES_ZONE_HEIGHT)) /
                            MAP_HEIGHT
                        }%`,
                    }}
                >
                    <div className="w-1/2 border-4 border-slate-800">
                        <p className="text-center text-sm">Excavation</p>
                    </div>
                    <div className="w-1/2 border-4 border-slate-800 border-l-0">
                        <p className="text-center text-sm">Construction</p>
                    </div>
                </div>
                <Obstacle x={295} y={260} radius={20} isHole={false} />
                <Obstacle x={340} y={260} radius={20} isHole={false} />
                <Obstacle x={385} y={260} radius={20} isHole={false} />
                <Obstacle x={430} y={260} radius={20} isHole={false} />
                <Obstacle x={475} y={260} radius={20} isHole={false} />
                <Obstacle x={520} y={260} radius={20} isHole={false} />

                <Obstacle x={50} y={400} radius={40} isHole={true} />
                <RobotPath
                    path={[
                        [565, 80],
                        [170, 80],
                        [170, 425],
                        [600, 425],
                    ]}
                />
            </div>
        </div>
    );
}
