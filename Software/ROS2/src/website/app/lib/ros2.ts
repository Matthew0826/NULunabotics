import * as rclnodejs from "rclnodejs";
import { sendToClient } from "./sockets";
import { Point as Vector2 } from "@/app/ui/map/robot-path";
import { GamepadState } from "../ui/dashboard/gamepad-state-provider";
const Plan = rclnodejs.require("lunabotics_interfaces/action/Plan");

export const lidarPoints: Point[] = [];

export type Point = {
    distance: number;
    angle: number;
    weight: number;
};

const LUNABOTICS_LIDAR_ROTATION_TYPE = "lunabotics_interfaces/msg/LidarRotation";
const LUNABOTICS_MOTORS_TYPE = "lunabotics_interfaces/msg/Motors";
const LUNABOTICS_PATH_TYPE = "lunabotics_interfaces/srv/Path";
const LUNABOTICS_OBSTACLE_TYPE = "lunabotics_interfaces/msg/Obstacle";
const LUNABOTICS_PLAN_TYPE = "lunabotics_interfaces/action/Plan";
const LUNABOTICS_POINT_TYPE = "lunabotics_interfaces/msg/Point";
const LUNABOTICS_RECT_TYPE = "lunabotics_interfaces/msg/Rect";
const LUNABOTICS_PATH_VISUAL_TYPE = "lunabotics_interfaces/msg/PathVisual";
const LUNABOTICS_EXCAVATOR_TYPE = "lunabotics_interfaces/msg/Excavator";
const LUNABOTICS_EXCAVATOR_PERCENT_TYPE = "lunabotics_interfaces/msg/ExcavatorPotentiometer";
const LUNABOTICS_ORIENTATION_CORRECTION_TYPE = "lunabotics_interfaces/msg/AccelerometerCorrection";
const LUNABOTICS_POWER_TYPE = "lunabotics_interfaces/msg/PowerData";
const ROS2_FLOAT_TYPE = "std_msgs/msg/Float32";
const ROS2_BOOL_TYPE = "std_msgs/msg/Bool";
const ROS2_STRING_TYPE = "std_msgs/msg/String";
const ROS2_BATTERY_TYPE = "sensor_msgs/msg/BatteryState";
const LUNABOTICS_SERIAL_PORT_TYPE = "lunabotics_interfaces/msg/SerialPortStates";
const LUNABOTICS_CONFIG_TYPE = "lunabotics_interfaces/msg/Config";

const pastMotorSpeeds: Record<string, number> = {};
const verifyMotorSpeed = (motor: string, speed: number) => {
    let motorSpeed = speed;
    // check if not much has changed
    if (Math.abs(pastMotorSpeeds[motor] - motorSpeed) < 0.05) {
        // make sure a small value is not being sent
        if (Math.abs(motorSpeed) < 0.1) {
            motorSpeed = 0.0;
            pastMotorSpeeds[motor] = motorSpeed;
            return motorSpeed;
        }
        pastMotorSpeeds[motor] = motorSpeed;
        return null;
    } else {
        pastMotorSpeeds[motor] = motorSpeed;
        return motorSpeed;
    }
}

export const publishMotorsToROS2 = (messageJson: GamepadState) => {
    const motorsMsg = rclnodejs.createMessageObject(LUNABOTICS_MOTORS_TYPE);
    const x = messageJson.x;
    const y = messageJson.y;
    const left = y + x;
    const right = y - x;
    const maxMagnitude = Math.max(1, Math.abs(left), Math.abs(right));
    let leftSpeed = verifyMotorSpeed("left_wheel", left / maxMagnitude);
    let rightSpeed = verifyMotorSpeed("right_wheel", right / maxMagnitude);

    if (leftSpeed != null && rightSpeed != null) {
        motorsMsg.front_left_wheel = leftSpeed;
        motorsMsg.back_left_wheel = leftSpeed;
        motorsMsg.front_right_wheel = rightSpeed;
        motorsMsg.back_right_wheel = rightSpeed;
        rosControlsPublisher.publish(motorsMsg);
    }

    const excavatorMsg = rclnodejs.createMessageObject(LUNABOTICS_EXCAVATOR_TYPE);
    const excavatorSpeed = messageJson.actuatorPower;
    const actuatorSpeed = verifyMotorSpeed("actuator", messageJson.isActuator ? excavatorSpeed : 0.0);
    const excavatorLifterSpeed = verifyMotorSpeed("excavator_lifter", messageJson.isActuator ? 0.0 : excavatorSpeed);
    const conveyorSpeed = verifyMotorSpeed("conveyor", messageJson.conveyorSpeed);

    if (actuatorSpeed != null && excavatorLifterSpeed != null && conveyorSpeed != null) {
        excavatorMsg.actuator_speed = actuatorSpeed;
        excavatorMsg.excavator_lifter_speed = excavatorLifterSpeed;
        excavatorMsg.conveyor_speed = conveyorSpeed;
        rosExcavatorPublisher.publish(excavatorMsg);
    }
};

export const publishMockObstacle = (messageJson: any) => {
    const obstacleMsg = rclnodejs.createMessageObject(LUNABOTICS_OBSTACLE_TYPE);
    obstacleMsg.position.x = messageJson.x;
    obstacleMsg.position.y = messageJson.y;
    obstacleMsg.radius = messageJson.radius;
    // Make sure the pathfinder thinks its high confidence since this is a mock
    for (let i = 0; i < 10; i++) {
        rosMockObstaclePublisher.publish(obstacleMsg);
    }
};

export const publishSimReset = (messageJson: any) => {
    const resetMsg = rclnodejs.createMessageObject(ROS2_BOOL_TYPE);
    resetMsg.data = true;
    rosSimResetPublisher.publish(resetMsg);
}

export const publishConfig = (messageJson: any) => {
    const configMsg = rclnodejs.createMessageObject(LUNABOTICS_CONFIG_TYPE);
    configMsg.node = messageJson.node;
    configMsg.category = messageJson.category;
    configMsg.setting = messageJson.setting;
    configMsg.value = messageJson.value;
    rosConfigPublisher.publish(configMsg);
};

export const publishCodeUpload = (messageJson: any) => {
    const codeUploadMsg = rclnodejs.createMessageObject(ROS2_STRING_TYPE);
    codeUploadMsg.data = messageJson.port;
    rosCodeUploadPublisher.publish(codeUploadMsg);
}

export const publishOrientationCorrection = (messageJson: any) => {
    const orientationMsg = rclnodejs.createMessageObject(LUNABOTICS_ORIENTATION_CORRECTION_TYPE);
    orientationMsg.initial_angle = messageJson.orientationCorrection;
    orientationMsg.should_reset = false;
    rosOrientationCorrectionPublisher.publish(orientationMsg);
}

export const sendPathfindingRequest = async (point1: Vector2, point2: Vector2, callback: (point: any[]) => void) => {
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
    rosClient.sendRequest(request, (response: any) => {
        callback(response.nodes);
    });
};

let goalHandle: rclnodejs.ClientGoalHandle<typeof LUNABOTICS_PLAN_TYPE>;

export async function sendPlanAction(start: any, should_excavate: boolean, should_dump: boolean, destination: any) {
    console.log('Waiting for action server...');
    await planActionClient.waitForServer(2000);

    const goal = new Plan.Goal();
    goal.start = start;
    goal.should_excavate = should_excavate;
    goal.should_dump = should_dump;
    goal.destination = destination;

    console.log('Sending goal request...');

    goalHandle = await planActionClient.sendGoal(goal, (feedback: rclnodejs.lunabotics_interfaces.action.Plan_Feedback) =>
        console.log(`Received feedback: ${feedback.progress}`)
    );

    if (!goalHandle.isAccepted()) {
        console.log('Goal rejected');
        return;
    }

    console.log('Goal accepted');

    // // Start a 2 second timer
    // let timer: rclnodejs.Timer;
    // timer = rosNode.createTimer(BigInt(2000000), () =>
    //     timerCallback(goalHandle, timer)
    // );
    const result = await goalHandle.getResult();

    if (goalHandle.isSucceeded()) {
        console.log(`Goal succeeded in ${result.time_elapsed_millis} ms`);
    } else {
        console.log(`Goal failed with status: ${goalHandle.status}`);
    }
    return result;
}
let doLoopAction = false;

function loopActionForever(excavate: boolean) {
    if (!doLoopAction) return;
    let promise;
    promise = sendPlanAction(robotPosition, excavate, !excavate, { x: -1, y: -1 });
    // wait for the promise to resolve
    promise.then((response: any) => {
        loopActionForever(!excavate);
    });

}

export function startLoopingAction(excavate: boolean) {
    doLoopAction = true;
    loopActionForever(excavate);
}

export async function stopLoopingAction() {
    if (!doLoopAction || !goalHandle) return;
    doLoopAction = false;
    const response = await goalHandle.cancelGoal();

    if (response.goals_canceling.length > 0) {
        console.log('Goal successfully canceled');
    } else {
        console.log('Goal failed to cancel');
    }
}

async function timerCallback(goalHandle: rclnodejs.ClientGoalHandle<typeof LUNABOTICS_PLAN_TYPE>, timer: rclnodejs.Timer) {
    console.log('Canceling goal');
    // Cancel the timer
    timer.cancel();

    const response = await goalHandle.cancelGoal();

    if (response.goals_canceling.length > 0) {
        console.log('Goal successfully canceled');
    } else {
        console.log('Goal failed to cancel');
    }
}


let rosControlsPublisher: rclnodejs.Publisher<typeof LUNABOTICS_MOTORS_TYPE>;
let rosExcavatorPublisher: rclnodejs.Publisher<typeof LUNABOTICS_EXCAVATOR_TYPE>;
let rosMockObstaclePublisher: rclnodejs.Publisher<typeof LUNABOTICS_OBSTACLE_TYPE>;
let rosSimResetPublisher: rclnodejs.Publisher<typeof ROS2_BOOL_TYPE>;
let rosConfigPublisher: rclnodejs.Publisher<typeof LUNABOTICS_CONFIG_TYPE>;
let rosCodeUploadPublisher: rclnodejs.Publisher<typeof ROS2_STRING_TYPE>;
let rosOrientationCorrectionPublisher: rclnodejs.Publisher<typeof LUNABOTICS_ORIENTATION_CORRECTION_TYPE>;
let rosClient: rclnodejs.Client<typeof LUNABOTICS_PATH_TYPE>;
let planActionClient: rclnodejs.ActionClient<typeof LUNABOTICS_PLAN_TYPE>;

let robotPosition = { x: 448, y: 100 };

rclnodejs.init().then(() => {
    const node = new rclnodejs.Node("website_backend");
    rosControlsPublisher = node.createPublisher(
        LUNABOTICS_MOTORS_TYPE,
        "physical_robot/motors"
    );
    rosMockObstaclePublisher = node.createPublisher(
        LUNABOTICS_OBSTACLE_TYPE,
        "navigation/obstacles"
    );
    rosSimResetPublisher = node.createPublisher(
        ROS2_BOOL_TYPE,
        "website/reset"
    );
    rosExcavatorPublisher = node.createPublisher(
        LUNABOTICS_EXCAVATOR_TYPE,
        "/physical_robot/excavator"
    );
    rosConfigPublisher = node.createPublisher(
        LUNABOTICS_CONFIG_TYPE,
        "website/config"
    );
    rosCodeUploadPublisher = node.createPublisher(
        ROS2_STRING_TYPE,
        "website/arduino_upload"
    );
    rosOrientationCorrectionPublisher = node.createPublisher(
        LUNABOTICS_ORIENTATION_CORRECTION_TYPE,
        "sensors/accelerometer_correction"
    );
    node.createSubscription(
        LUNABOTICS_LIDAR_ROTATION_TYPE,
        "sensors/lidar",
        (msg: any) => {
            sendToClient("lidar", msg.points, true);
        }
    );
    let previousObstacle: any = null;
    node.createSubscription(
        LUNABOTICS_OBSTACLE_TYPE,
        "navigation/obstacles",
        (msg: any) => {
            if (msg == previousObstacle) return;
            sendToClient("obstacles", msg);
            previousObstacle = msg;
        }
    );
    node.createSubscription(
        LUNABOTICS_POINT_TYPE,
        "sensors/position",
        (msg: any) => {
            robotPosition.x = msg.x;
            robotPosition.y = msg.y;
            sendToClient("position", msg, false);
        }
    );
    node.createSubscription(
        LUNABOTICS_RECT_TYPE,
        "/sensors/position_confidence",
        (msg: any) => {
            sendToClient("position_confidence", msg, true);
        }
    );
    node.createSubscription(
        ROS2_FLOAT_TYPE,
        "sensors/orientation",
        (msg: any) => {
            sendToClient("orientation", msg.data, true);
        }
    );
    node.createSubscription(
        LUNABOTICS_PATH_VISUAL_TYPE,
        "navigation/path",
        (msg: any) => {
            sendToClient("path", msg.nodes);
        }
    );
    node.createSubscription(
        LUNABOTICS_PATH_VISUAL_TYPE,
        "navigation/odometry_path",
        (msg: any) => {
            sendToClient("odometry_path", msg.nodes);
        }
    );
    node.createSubscription(
        ROS2_BOOL_TYPE,
        "website/reset",
        (msg: any) => {
            sendToClient("reset", msg.data);
        }
    );
    node.createSubscription(
        LUNABOTICS_SERIAL_PORT_TYPE,
        "sensors/serial_port_state",
        (msg: any) => {
            sendToClient("serial_port_state", msg);
        }
    );
    node.createSubscription(
        LUNABOTICS_EXCAVATOR_TYPE,
        "physical_robot/excavator",
        (msg: any) => {
            sendToClient("excavator", msg, true);
        }
    )
    node.createSubscription(
        LUNABOTICS_MOTORS_TYPE,
        "physical_robot/motors",
        (msg: any) => {
            sendToClient("motors", msg, false);
        }
    )
    node.createSubscription(
        LUNABOTICS_EXCAVATOR_PERCENT_TYPE,
        "sensors/excavator_percent",
        (msg: any) => {
            sendToClient("excavator_percent", msg, true);
        }
    )
    node.createSubscription(
        ROS2_FLOAT_TYPE,
        "sensors/excavator_distance",
        (msg: any) => {
            sendToClient("distance_sensor", msg, true);
        }
    )
    node.createSubscription(
        ROS2_BATTERY_TYPE,
        "battery/state",
        (msg: any) => {
            sendToClient("battery", msg, true);
        }
    )
    rosClient = node.createClient(
        LUNABOTICS_PATH_TYPE,
        'pathfinder'
    );
    planActionClient = new rclnodejs.ActionClient(
        node,
        LUNABOTICS_PLAN_TYPE,
        'plan'
    );
    node.spin();
});
