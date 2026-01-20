import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation as R

qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, depth=10)


def quaternion_to_yaw(qx, qy, qz, qw):
    rotation = R.from_quat([qx, qy, qz, qw])

    _, _, yaw = rotation.as_euler("xyz", degrees=True)

    yaw_normalized = yaw % 360
    return yaw_normalized


class RotationWaypointsFollower(Node):
    def __init__(self):
        super().__init__(node_name="rotation_waypoints_follower")
        self._velocity_pub = self.create_publisher(Twist, "/nav_vel", qos_profile=10)
        self._ground_truth_sub = self.create_subscription(
            Odometry, "/ground_truth_odom", self._update_ground_odom, qos_profile=10
        )
        self._odom_sub = self.create_subscription(Odometry, "/odometry/filtered", self._update_odom, qos_profile=10)

        self._x = 0
        self._y = 0
        self._angle = 0
        self._rot_error = []
        self._tolerance = 0.075

    def _rotate_angle(self, deg):
        msg = Twist()

        while True:
            # self.get_logger().info(f"Current angle: {self._angle}")
            error = deg - self._angle
            if error > 180:
                error -= 360
            elif error < -180:
                error += 360

            if abs(error) < self._tolerance:
                self._velocity_pub.publish(Twist())
                break

            angular_speed = 0.025
            msg.angular.z = angular_speed if error > 0 else -angular_speed
            self._velocity_pub.publish(msg)
            time.sleep(0.01)

    def _update_ground_odom(self, msg: Odometry):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y

        quat = msg.pose.pose.orientation
        self._angle = quaternion_to_yaw(quat.x, quat.y, quat.z, quat.w)

    def _update_odom(self, msg: Odometry):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y

        quat = msg.pose.pose.orientation
        angle = quaternion_to_yaw(quat.x, quat.y, quat.z, quat.w)
        error = pow(self._angle - angle, 2)
        self.get_logger().info(
            f"Current angle: {self._angle:.3f} deg\tMeasured angle: {angle:.3f} deg\tError: {error:.3f}"
        )
        self._rot_error.append(error)

    def run(self):
        # Example angles
        angles = [270, 180, 195, 150, 90, 120, 45]

        for angle in angles:
            self._rotate_angle(angle)
            self.get_logger().info("Angle achieved!")
            self.get_logger().info(f"MSE after rotation {sum(self._rot_error) / len(self._rot_error):.3f}")
            time.sleep(5)

        self.get_logger().info("Finished")


def main():
    rclpy.init()
    node = RotationWaypointsFollower()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    time.sleep(2)
    node.run()
    node.destroy_node()
    executor.remove_node(node)
    executor.shutdown()
    spin_thread.join()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
