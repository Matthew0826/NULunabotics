import * as rclnodejs from "rclnodejs";

const ROS2_STRING_TYPE = "std_msgs/msg/String";
const ROS2_UINT16_MULTI_ARRAY_TYPE = "std_msgs/msg/UInt16MultiArray";

let rosPublisher: rclnodejs.Publisher<typeof ROS2_STRING_TYPE>;
let rosSubscriber: rclnodejs.Subscription;

export const lidarPoints: { distance: number; angle: number }[] = [];

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
            // data follows this pattern:
            // distance1, angle1, distance2, angle2, distance3, angle3
            // so we need to split it into pairs
            lidarPoints.length = 0;
            for (let i = 0; i < data.length; i += 2) {
                const distance = data[i];
                const angle = (Math.PI * (data[i + 1] / 100)) / 180;
                lidarPoints.push({ distance, angle });
            }
        }
    );
    node.spin();
});
