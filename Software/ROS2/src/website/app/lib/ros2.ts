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

export const publishToROS2 = (message: string) => {
    const messageJson = JSON.parse(message);
    const motorsMsg = rclnodejs.createMessageObject(LUNABOTICS_MOTORS_TYPE);
    motorsMsg.front_left_wheel = messageJson.y1;
    motorsMsg.back_left_wheel = messageJson.y1;
    motorsMsg.front_right_wheel = messageJson.y2;
    motorsMsg.back_right_wheel = messageJson.y2;
    motorsMsg.conveyor = messageJson.buttonLeft ? 1 : 0;
    motorsMsg.outtake = messageJson.buttonRight ? 1 : 0;
    rosPublisher.publish(motorsMsg);
    console.log("Published: ", message);
};

export const sendPathfindingRequest = async (point1: Vector2, point2: Vector2, callback: (point: ROSPoint[]) => void ) => {
    const node = new rclnodejs.Node("pathfinding_request_node");
    const client = node.createClient(
        'lunabotics_interfaces/srv/Path',
        'pathfinding_request'
    );
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

    let result = await client.waitForService(1000);
    if (!result) {
        console.log('Error: service not available');
        rclnodejs.shutdown();
        return;
    }

    console.log(`Sending: ${typeof request}`, request);
    client.sendRequest(request, (response) => {
        console.log(`Result: ${typeof response}`, response);
        callback(response.nodes);
        rclnodejs.shutdown();
    });

    node.spin();
};

let rosPublisher: rclnodejs.Publisher<typeof LUNABOTICS_MOTORS_TYPE>;
let rosSubscriber: rclnodejs.Subscription;

rclnodejs.init().then(() => {
    const node = new rclnodejs.Node("website_backend");
    rosPublisher = node.createPublisher(
        LUNABOTICS_MOTORS_TYPE,
        "physical_robot/motors"
    );
    rosSubscriber = node.createSubscription(
        LUNABOTICS_LIDAR_ROTATION_TYPE,
        "sensors/lidar",
        (msg: any) => {
            const data = msg.points;
            lidarPoints.length = 0;
            for (let i = 0; i < data.length; i += 1) {
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
    node.spin();
});
