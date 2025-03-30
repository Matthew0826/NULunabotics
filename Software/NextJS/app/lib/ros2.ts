import * as rclnodejs from "rclnodejs";
import { sendToClient } from "../socket/route";

const ROS2_STRING_TYPE = "std_msgs/msg/String";
const ROS2_UINT16_MULTI_ARRAY_TYPE = "std_msgs/msg/UInt16MultiArray";

let rosPublisher: rclnodejs.Publisher<typeof ROS2_STRING_TYPE>;
let rosSubscriber: rclnodejs.Subscription;

export type Point = {
    distance: number;
    angle: number;
    weight: number;
};

export const lidarPoints: Point[] = [];

export function publishToROS2(message: string) {
    const stringMsgObject = rclnodejs.createMessageObject(ROS2_STRING_TYPE);
    stringMsgObject.data = message;
    rosPublisher.publish(stringMsgObject);
    console.log("Published: ", message);
}

rclnodejs.init().then(() => {
    const node = new rclnodejs.Node("website_backend");
    rosPublisher = node.createPublisher(ROS2_STRING_TYPE, "website/controller");
    rosSubscriber = node.createSubscription(
        ROS2_UINT16_MULTI_ARRAY_TYPE,
        "sensors/lidar",
        (msg: any) => {
            const data = msg.data;
            lidarPoints.length = 0;
            for (let i = 0; i < data.length; i += 3) {
                const weight = data[i];
                const distance = data[i + 2];
                const angle = (Math.PI * (data[i + 1] / 100)) / 180;
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
