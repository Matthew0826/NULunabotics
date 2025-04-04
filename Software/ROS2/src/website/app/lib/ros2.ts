import * as rclnodejs from "rclnodejs";
import { sendToClient } from "./sockets";

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
