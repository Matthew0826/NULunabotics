import * as rclnodejs from "rclnodejs";

const ROS2_STRING_TYPE = "std_msgs/msg/String";
const ROS2_FLOAT32_MULTI_ARRAY_TYPE = "std_msgs/msg/Float32MultiArray";

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
        ROS2_FLOAT32_MULTI_ARRAY_TYPE,
        "sensors/lidar",
        (msg: any) => {
            const data = msg.data;
            const distance = data[0];
            const angle = (Math.PI * data[1]) / 180;
            lidarPoints.push({ distance, angle });
            if (lidarPoints.length >= 360) {
                // Moving window
                lidarPoints.shift();
            }
        }
    );
    node.spin();
});
