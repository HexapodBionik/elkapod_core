import rclpy
from .elkapod_comm_server import ElkapodCommServer


class MyRobotDriver:
    def init(self, webots_node, properties):
        self._robot = webots_node.robot

        rclpy.init(args=None)
        self._node = ElkapodCommServer(self._robot)

    def step(self):
        rclpy.spin_once(self._node, timeout_sec=0)