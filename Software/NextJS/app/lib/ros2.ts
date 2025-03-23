import * as rclnodejs from "rclnodejs";

const ROS2_STRING_TYPE = "std_msgs/msg/String";

let rosPublisher: rclnodejs.Publisher<typeof ROS2_STRING_TYPE>;

export function publishToROS2(message: string) {
    const stringMsgObject = rclnodejs.createMessageObject(ROS2_STRING_TYPE);
    stringMsgObject.data = message;
    rosPublisher.publish(stringMsgObject);
    console.log("Published: ", message);
}

rclnodejs.init().then(() => {
    const node = new rclnodejs.Node("website_backend");
    rosPublisher = node.createPublisher(ROS2_STRING_TYPE, "website/controller");
    node.spin();
});
