import { MapPoint, RelativeLidarPoint } from "@/app/types/map-objects";
import { ROSMapPoint, ROSObstacle } from "@/app/types/ros";
import { ConfigType } from "@/app/types/config";

export type ROSSocketMessage = ({
    type: "lidar";
    message: RelativeLidarPoint[];
}
    | {
        type: "motors";
        message: {
            back_left_wheel: number;
            back_right_wheel: number;
            front_left_wheel: number;
            front_right_wheel: number;
        };
    }
    | {
        type: "excavator";
        message: {
            excavator_lifter_speed?: number;
            conveyor_speed?: number;
            actuator_speed?: number;
        };
    }
    | {
        type: "excavator_percent";
        message: {
            excavator_lifter_percent: number;
            actuator_percent: number;
        };
    }
    | {
        type: "distance_sensor";
        message: {
            data: number;
        };
    }
    | {
        type: "obstacles";
        message: ROSObstacle[];
    }
    | {
        type: "position";
        message: {
            x: number;
            y: number;
        };
    }
    | {
        type: "orientation";
        message: number;
    }
    | {
        type: "position_confidence";
        message: {
            x1: number;
            y1: number;
            x2: number;
            y2: number;
        };
    }
    | {
        type: "path";
        message: ROSMapPoint[];
    }
    | {
        type: "odometry_path";
        message: ROSMapPoint[];
    }
    | {
        type: "battery";
        message: {
            voltage: number;
            current: number;
            percentage: number;
        }[];
    }
    | {
        type: "resetAutonomous";
        message?: any;
    }
    | {
        type: "loadConfigProfile";
        message: {
            profile: string;
            config: ConfigType[];
        };
    }
    | {
        type: "saveConfigProfile";
        message: any;
    }
    | {
        type: "uploadCode";
        message: any;
    }
    | {
        type: "controls";
        message: any;
    }
    | {
        type: "serial_port_state";
        message: {
            states: SerialPortType[]
        };
    }
    | {
        type: "orientationCorrection";
        message: any;
    }
    | {
        type: "mockObstacle";
        message: {
            point: MapPoint;
        };
    }
    | {
        type: "sendPathfindingRequest";
        message: {
            point1: MapPoint;
            point2: MapPoint;
            robot: ROSMapPoint;
        };
    }
    | {
        type: "beginAutonomous" | "stopAutonomous";
    });
