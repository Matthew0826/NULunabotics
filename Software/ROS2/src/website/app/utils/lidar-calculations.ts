import * as THREE from 'three';
import {RelativeLidarOrigin} from "@/app/types/robot";

/**
 * Converts polar coordinates in a pitched local XZ-plane into world space.
 *
 * @param distance - Distance from the origin
 * @param angle - Angle within the rotated local XZ-plane (in radians)
 * @param lidarOrigin - The origin of the lidar in robot relative coordinates
 * @returns THREE.Vector3 - World position of the point
 */
export function lidarRelativeToRelative3D(
    distance: number,
    angle: number,
    lidarOrigin: RelativeLidarOrigin,
    scale = 1
): THREE.Vector3 {
    const euler = new THREE.Euler(lidarOrigin.pitch, lidarOrigin.yawOffset, 0, 'XYZ');

    const localDir = new THREE.Vector3(
        Math.sin(angle),
        0,
        Math.cos(angle)
    );

    const worldDir = localDir.applyEuler(euler);

    return worldDir.multiplyScalar(distance * scale).add(new THREE.Vector3(
        lidarOrigin.relPos.x, lidarOrigin.relPos.y, lidarOrigin.relPos.z
    ));
}

