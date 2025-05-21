import {Dispatch, SetStateAction} from "react";
import {GlobalLidarPoint, ObstacleType} from "@/app/types/map-objects";
import * as THREE from 'three';

export type RobotContextType = {
    leftWheelSpeed: number;
    rightWheelSpeed: number;
    setLeftWheelSpeed: Dispatch<SetStateAction<number>>;
    setRightWheelSpeed: Dispatch<SetStateAction<number>>;
    excavatorPosition: number;
    setExcavatorPosition: Dispatch<SetStateAction<number>>;
    lidarPoints: GlobalLidarPoint[][];
    setLidarPoints: Dispatch<SetStateAction<GlobalLidarPoint[][]>>;
    lidarRelativePoints: RelativeLidarPoint[];
    setLidarRelativePoints: Dispatch<SetStateAction<RelativeLidarPoint[]>>;
    lidarOrigin: RelativeLidarOrigin;
    obstacles: ObstacleType[];
    setObstacles: Dispatch<SetStateAction<ObstacleType[]>>;
    robot: RobotPosition;
    setRobot: Dispatch<SetStateAction<RobotPosition>>;
};

export type RobotPosition = {
    x: number;
    y: number;
    width: number; // in meters
    height: number; // in meters
    rotation: number; // in degrees
    posConfidenceRect: {
        x1: number;
        y1: number;
        x2: number;
        y2: number;
    };
}

export type RelativeLidarOrigin = {
    relPos: THREE.Vector3;
    pitch: number; // The angle the lidar is looking (in radians) (0 is straight ahead)
    yawOffset: number; // Offset for the direction the field of view is pointed.
}

export type Rect = {
    x1: number;
    y1: number;
    x2: number;
    y2: number;
}
