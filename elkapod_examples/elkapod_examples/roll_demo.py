import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from rclpy.executors import SingleThreadedExecutor
import numpy as np


class RollDemo(Node):
    def __init__(self):
        super().__init__("roll_demo")
        self._publisher = self.create_publisher(Float64, "/roll_setpoint", 10)

        self._roll_setpoints_deg = [0.0, 1.0, 3.0, 2.0, 1.5, 0.0, 0.0, -1.0, -3.0, -2.0, -1.5, 0.0, 0.0]
        self._generator = self._setpoints()
        self._current_setpoint = 0.0
        self._start_clock = self.get_clock().now().seconds_nanoseconds()[0]
        self._change_interval_sec = 10.0

        self._my_timer = self.create_timer(0.02, self._run)

        self.get_logger().info("Roll demo node started!")

    def _setpoints(self):
        for setpoint in self._roll_setpoints_deg:
            print(setpoint)
            setpoint_rad = setpoint * np.pi / 180
            yield setpoint_rad

    def _run(self):
        if self.get_clock().now().seconds_nanoseconds()[0] - self._start_clock > self._change_interval_sec:
            try:
                self._current_setpoint = next(self._generator)
            except StopIteration:
                self.get_logger().info("All setpoints published! Stopping...")
                self._my_timer.cancel()
            self._start_clock = self.get_clock().now().seconds_nanoseconds()[0]
            self.get_logger().info(f"Next setpoint: {self._current_setpoint / np.pi * 180:.2f} deg")

        msg = Float64()
        msg.data = self._current_setpoint
        self._publisher.publish(msg)



def main():
    rclpy.init()
    node = RollDemo()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()