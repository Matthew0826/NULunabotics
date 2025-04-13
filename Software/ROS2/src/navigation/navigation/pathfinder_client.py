import rclpy
from rclpy.node import Node

from lunabotics_interfaces.srv import Path

class PathfinderClient(Node):
    """Used to request paths from the pathfinder service."""
    def __init__(self):
        super().__init__('pathfinder_client')
        self.pathfinder_client = self.create_client(Path, 'pathfinder')
        while not self.pathfinder_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('pathfinder service not available, waiting again...')
        self.req = Path.Request()
    
    def make_path(self, start, end):
        self.req.start = start
        self.req.end = end
        return self.pathfinder_client.call_async(self.req)


def use_pathfinder(pathfinder_client, start, end):
    future = pathfinder_client.make_path(start, end)
    # https://docs.ros.org/en/jazzy/How-To-Guides/Sync-Vs-Async.html
    # TODO: Maybe this will cause deadlock
    rclpy.spin_until_future_complete(pathfinder_client, future)
    response = future.result()
    # pathfinder_client.get_logger().info("Pathfinder response:" + str(response.nodes))
    return response.nodes