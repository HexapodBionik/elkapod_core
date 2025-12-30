import numpy as np
import rclpy
from rclpy.node import Node, ParameterDescriptor
from rclpy.qos import qos_profile_default
from std_msgs.msg import Float64


class OrientationDemo(Node):
    def __init__(self, node_name: str, topic_name: str, default_trajectory: list):
        super().__init__(node_name)
        self._publisher = self.create_publisher(Float64, topic_name, qos_profile_default)
        self.declare_parameter(
            "change_interval",
            10.0,
            descriptor=ParameterDescriptor(description="Time between two setpoints consecutive in seconds"),
        )

        self.declare_parameter(
            "setpoints",
            default_trajectory,
            descriptor=ParameterDescriptor(description="Setpoints trajectory given in degrees"),
        )

        self._roll_setpoints_deg = self.get_parameter("setpoints").get_parameter_value().double_array_value.tolist()
        self._roll_setpoints_deg.append(0.0)
        self._change_interval_sec = self.get_parameter("change_interval").get_parameter_value().double_value

        self._generator = self._setpoints()
        self._current_setpoint = 0.0

        self._start_clock = self.get_clock().now().seconds_nanoseconds()[0]
        self._my_timer = self.create_timer(0.02, self._run)
        self.get_logger().info("Node started!")

    def _send_setpoint(self, setpoint: float):
        msg = Float64()
        msg.data = setpoint
        self._publisher.publish(msg)

    def _setpoints(self):
        for setpoint in self._roll_setpoints_deg:
            setpoint_rad = np.deg2rad(setpoint)
            yield setpoint_rad

    def _run(self):
        self._send_setpoint(self._current_setpoint)
        if self.get_clock().now().seconds_nanoseconds()[0] - self._start_clock > self._change_interval_sec:
            try:
                self._current_setpoint = next(self._generator)
                self._start_clock = self.get_clock().now().seconds_nanoseconds()[0]
                self.get_logger().info(f"Next setpoint: {self._current_setpoint / np.pi * 180:.2f} deg")
            except StopIteration:
                self.get_logger().info("All setpoints published! Stopping...")
                self._my_timer.cancel()
                rclpy.try_shutdown()
