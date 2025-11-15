export type ROSObstacle = {
    position: {
        x: number;
        y: number;
    }
    radius: number;
    is_rock: boolean;
    relative_position: {
        x: number;
        y: number;
    }
}

export type ROSMapPoint = {
    x: number;
    y: number;
}
