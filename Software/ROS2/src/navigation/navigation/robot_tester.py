import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from lunabotics_interfaces.action import SelfDriver


class RobotTester(Node):

    def __init__(self):
        super().__init__('robot_tester')
        self._action_client = ActionClient(self, SelfDriver, 'self_driver')

    def send_goal(self, order):
        goal_msg = SelfDriver.Goal()
        goal_msg.target

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    future = action_client.send_goal(10)

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()
