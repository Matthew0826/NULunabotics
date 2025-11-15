import {Vector3} from "three";

export type MapPoint = [number, number];

export type GlobalLidarPoint = {
    globPos: Vector3;
    weight: number;
}

export type RelativeLidarPoint = {
    distance: number;
    angle: number;
    weight: number;
}

export type ObstacleType = {
    x: number;
    y: number;
    radius: number;
    isHole: boolean;
    relativeX: number;
    relativeY: number;
};
