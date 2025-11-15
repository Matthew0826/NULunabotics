import * as THREE from 'three';
import {RelativeLidarOrigin} from "@/app/types/robot";

/**
 * Converts polar coordinates in a pitched local XZ-plane into world space.
 *
 * @param distance - Distance from the origin
 * @param angle - Angle within the rotated local XZ-plane (in radians)
 * @param lidarOrigin - The origin of the lidar in robot relative coordinates
 * @param robotYawDeg - The yaw of the robot in degrees
 * @returns THREE.Vector3 - World position of the point
 */
export function lidarRelativeToRelative3D(
    distance: number,
    angle: number,
    lidarOrigin: RelativeLidarOrigin,
    robotYawDeg = 0, // robot.rotation in degrees
    scale = 1
): THREE.Vector3 {
    // Convert robot rotation from degrees to radians
    const robotYawRad = THREE.MathUtils.degToRad(robotYawDeg);

    // Lidar pitch and yaw offset in robot's local frame
    const euler = new THREE.Euler(lidarOrigin.pitch, lidarOrigin.yawOffset, 0, 'XYZ');

    // Local direction based on lidar angle
    const localDir = new THREE.Vector3(
        Math.sin(angle),
        0,
        Math.cos(angle)
    );

    // Apply lidar's pitch/yaw
    const worldDir = localDir.applyEuler(euler).multiplyScalar(distance * scale);

    // Translate to lidar origin
    const relativePoint = worldDir.add(lidarOrigin.relPos.clone());

    // Now rotate around the Y axis centered on origin (0,0,0) by robot yaw
    return relativePoint.applyAxisAngle(new THREE.Vector3(0, 1, 0), robotYawRad);
}
