import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from lunabotics_intefaces.action import SelfDriver

# listen for motor updates
from lunabotics_interfaces.msg import Motor

# publish new coordinates and rotation
from lunabotics_interfaces.msg import LidarRotation, Point

class RobotSimulator(Node):
    def __init__(self):
        super().__init__('self_driver_action_client')
        
        self._action_client = ActionClient(self, SelfDriver, 'self_driver')

        # fields
        # speed is how fast in centimeters per second it goes with max power
        self.speed = 1
        self.position = Point
        self.position.x = 0
        self.position.y = 0
        
        # create it as action client
        self._action_client = ActionClient(
                 self, 
                 SelfDriver, 
                 'self_driver'
                 )


        # create subscription to motors
        self.subscription = self.create_subscription(
                Motors,
                'website/controller',
                self.on_motor_event,
                10)

        # create it as publisher for position
        self.position_pub = self.create_publisher(
                self,
                Point,
                'sensors/position',
                10
                )

        # create it as publisher for orientation
        self.orientation_pub = self.create_publisher(
                self,
                Float32,
                'sensors/orientation',
                'on_orientation'
                )

        # hide unused variable warning (useless)
        self.subscription
        self.position_pub
        self.orientation_pub

    def send_goal(self, order):
        goal_msg = SelfDriver.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

    def on_motor_event(self, msg):
        # increment current position by motor power
        self.position.x += self.speed * msg.front_left_wheel
        self.position.y += self.speed * msg.front_right_wheel

        # position to publish
        msg = self.position

        # publish new position
        self.motor_pub.publish(msg)

        # orientation to publish
        msg = 0.0
    
        # publish new orientation
        self.orientation_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    robot_simulator = RobotSimulator()
    rclpy.spin(robot_simulator)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
