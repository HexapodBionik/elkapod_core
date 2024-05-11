import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from elkapod_msgs.msg import LegPositions, LegFrames, LegFrame
from .utils import load_parameters
from elkapod_algorithms.kinematics.kinematics_solvers import KinematicsSolver
from elkapod_algorithms.kinematics.kinematics_exceptions import PointOutOfReach


class ElkapodKinematics(Node):
    def __init__(self):
        super().__init__("elkapod_kinematics")
        self.declare_parameter("config_path")
        path = self.get_parameter("config_path").get_parameter_value().string_value
        m1, a1, a2, a3, soft_ang1, soft_ang2, soft_ang3 = load_parameters(path)

        self._leg_positions = [[0.0, 0.0, 0.0] for _ in range(6)]

        # TODO Will need an update when mount angles are added to solver
        self._kinematics_solver = KinematicsSolver(m1, a1, a2, a3)

        self._soft_angle1 = soft_ang1
        self._soft_angle2 = soft_ang2
        self._soft_angle3 = soft_ang3

        self._elkapod_legs_positions_subscription = self.create_subscription(LegPositions,
                                                                             "elkapod_legs_goals",
                                                                             self._leg_positions_callback,
                                                                             10)

        self._elkapod_leg_frames_publisher = self.create_publisher(LegFrames,
                                                                   "elkapod_comm_server_leg_frames",
                                                                   10)

        self._leg_positions_publisher = self.create_publisher(LegPositions,
                                                              "elkapod_legs_positions",
                                                              10)
        self._timer_period = 0.05
        self._leg_position_timer = self.create_timer(self._timer_period, self._publish_leg_positions)
        self.get_logger().info(
            f"Initialized ElkapodKinematics with software angles: {[self._soft_angle1, self._soft_angle2, self._soft_angle3]}")

    def _leg_positions_callback(self, msg):
        result = list()
        leg_positions = msg.leg_positions
        for i, leg in enumerate(leg_positions):
            result.append(self._point_to_leg_frame(i, leg))
        my_message = LegFrames()
        my_message.leg_frames = result
        self._elkapod_leg_frames_publisher.publish(my_message)

    def _point_to_leg_frame(self, leg_nb, point):
        message = LegFrame()
        message.leg_nb = leg_nb
        message.servo_op_codes = [1, 1, 1]
        p = [point.x, point.y, point.z]

        try:
            message.servo_angles = self._kinematics_solver.inverse(np.array(p)).tolist()

            # TODO This hardcoded values should be changed
            # message.servo_angles[1] += 17.5
            # message.servo_angles[2] += 40.7
            message.servo_angles[0] += self._soft_angle1
            message.servo_angles[1] += self._soft_angle2
            message.servo_angles[2] += self._soft_angle3
            self._leg_positions[leg_nb] = p
            return message
        except PointOutOfReach:
            self.get_logger().error(f"For leg {leg_nb}, given point {point.x},{point.y},{point.z} is out of range")

    def _publish_leg_positions(self):
        message = LegPositions()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "root"  # Not sure
        message.leg_positions = [self._list_to_point(p) for p in self._leg_positions]
        self._leg_positions_publisher.publish(message)

    @staticmethod
    def _list_to_point(point_data):
        return Point(x=point_data[0], y=point_data[1], z=point_data[2])


def main(args=None):
    rclpy.init(args=args)
    node = ElkapodKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
