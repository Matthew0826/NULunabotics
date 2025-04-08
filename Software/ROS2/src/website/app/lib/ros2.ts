import * as rclnodejs from "rclnodejs";
import { sendToClient } from "./sockets";
import { Point as Vector2 } from "@/app/ui/map/robot-path";
import { Point as ROSPoint } from "rclnodejs/lib/lunabotics_interfaces/msg/Point";

export const lidarPoints: Point[] = [];

export type Point = {
    distance: number;
    angle: number;
    weight: number;
};

const LUNABOTICS_LIDAR_ROTATION_TYPE =
    "lunabotics_interfaces/msg/LidarRotation";
const LUNABOTICS_MOTORS_TYPE = "lunabotics_interfaces/msg/Motors";
const LUNABOTICS_PATH_TYPE = "lunabotics_interfaces/srv/Path";
const LUNABOTICS_OBSTACLE_TYPE = "lunabotics_interfaces/msg/Obstacle";

export const publishToROS2 = (message: string) => {
    const messageJson = JSON.parse(message);
    const motorsMsg = rclnodejs.createMessageObject(LUNABOTICS_MOTORS_TYPE);
    motorsMsg.front_left_wheel = messageJson.y1;
    motorsMsg.back_left_wheel = messageJson.y1;
    motorsMsg.front_right_wheel = messageJson.y2;
    motorsMsg.back_right_wheel = messageJson.y2;
    motorsMsg.conveyor = messageJson.buttonLeft ? 1 : 0;
    motorsMsg.outtake = messageJson.buttonRight ? 1 : 0;
    rosControlsPublisher.publish(motorsMsg);
    console.log("Published: ", message);
};

export const sendPathfindingRequest = async (point1: Vector2, point2: Vector2, callback: (point: ROSPoint[]) => void ) => {
    // To view service events use the following command:
    //    ros2 topic echo "/add_two_ints/_service_event"
    // client.configureIntrospection(
    //     node.getClock(),
    //     rclnodejs.QoS.profileSystemDefault,
    //     rclnodejs.ServiceIntrospectionStates.METADATA
    // );

    const request = {
        start: {
            x: point1[0],
            y: point1[1],
        },
        end: {
            x: point2[0],
            y: point2[1],
        },
    };

    let result = await rosClient.waitForService(1000);
    if (!result) {
        console.log('Error: service not available');
        return;
    }

    console.log(`Sending: ${typeof request}`, request);
    rosClient.sendRequest(request, (response) => {
        callback(response.nodes);
    });
};

let rosControlsPublisher: rclnodejs.Publisher<typeof LUNABOTICS_MOTORS_TYPE>;
let rosLidarSubscriber: rclnodejs.Subscription;
let rosObstaclesSubscriber: rclnodejs.Subscription;
let rosClient: rclnodejs.Client<typeof LUNABOTICS_PATH_TYPE>;

rclnodejs.init().then(() => {
    const node = new rclnodejs.Node("website_backend");
    rosControlsPublisher = node.createPublisher(
        LUNABOTICS_MOTORS_TYPE,
        "physical_robot/motors"
    );
    rosLidarSubscriber = node.createSubscription(
        LUNABOTICS_LIDAR_ROTATION_TYPE,
        "sensors/lidar",
        (msg: any) => {
            const data = msg.points;
            lidarPoints.length = 0;
            for (let i = 0; i < data.length; i++) {
                const distance = data[i].distance;
                const angle = data[i].angle;
                const weight = data[i].weight;
                if (distance < 15000)
                    lidarPoints.push({ distance, angle, weight });
            }

            sendToClient(
                JSON.stringify([
                    {
                        graph: "Lidar",
                        dataSet: "Points",
                        newData: lidarPoints,
                    },
                ])
            );
        }
    );
    let previousObstacle: any = null;
    rosObstaclesSubscriber = node.createSubscription(
        LUNABOTICS_OBSTACLE_TYPE,
        "navigation/obstacles",
        (msg: any) => {
            if (msg == previousObstacle) return;
            sendToClient(
                JSON.stringify([
                    {
                        graph: "Obstacles",
                        dataSet: "Obstacles",
                        newData: msg,
                    },
                ])
            );
            previousObstacle = msg;
        }
    );
    rosClient = node.createClient(
        LUNABOTICS_PATH_TYPE,
        'pathfinder'
    );
    node.spin();
});
