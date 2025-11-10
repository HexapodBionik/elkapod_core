import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import threading

qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)

def quaternion_to_yaw(qx, qy, qz, qw):
    # Create a rotation object from the quaternion
    rotation = R.from_quat([qx, qy, qz, qw])

    # Convert to Euler angles (roll, pitch, yaw)
    roll, pitch, yaw = rotation.as_euler('xyz', degrees=True)

    # Normalize yaw to 0-360 range
    yaw_normalized = yaw % 360
    return yaw_normalized

class RotationWaypointsFollower(Node):
    def __init__(self):
        super().__init__(node_name="rotation_waypoints_follower")
        self._velocity_pub = self.create_publisher(Twist, "/cmd_vel", qos_profile=10)
        self._velocity_sub = self.create_subscription(Odometry, "/ground_truth_odom", self._update_odom, qos_profile=10)

        self._x = 0
        self._y = 0
        self._angle = 0
        self._tolerance = 0.075

    def _rotate_angle(self, deg):
        msg = Twist()

        while True:
            self.get_logger().info(f"Current angle: {self._angle}")
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

    def _update_odom(self, msg: Odometry):

        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y

        quat = msg.pose.pose.orientation
        self._angle = quaternion_to_yaw(quat.x, quat.y, quat.z, quat.w)

    def run(self):
        # Example angles
        angles = [270, 180, 195, 150, 90, 120, 45]

        for angle in angles:
            self._rotate_angle(angle)
            self.get_logger().info("Angle achieved!")
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


if __name__ == '__main__':
    main()
