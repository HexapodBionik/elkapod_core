import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from scipy.spatial.transform import Rotation as R
import threading
import time
import math


def quaternion_to_yaw(qx, qy, qz, qw):
    # Create a rotation object from the quaternion
    rotation = R.from_quat([qx, qy, qz, qw])

    # Convert to Euler angles (roll, pitch, yaw)
    roll, pitch, yaw = rotation.as_euler('xyz', degrees=True)

    # Normalize yaw to 0-360 range
    yaw_normalized = yaw % 360
    return yaw_normalized


class RectangleDrawer(Node):
    def __init__(self):
        super().__init__("rectangle_drawer")
        self.declare_parameter("side_length", value=1.0)
        self.declare_parameter("loops", value=1)
        self.declare_parameter("clockwise", value=False)

        self._true_position_subscriber = self.create_subscription(Odometry, "/ground_truth_odom", self._update_odom, 10)
        self._publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        self._start_clock = 0

        self._x = 0
        self._y = 0
        self._angle = 0

        self._true_x = 0
        self._true_y = 0
        self._true_angle = 0

        self._tolerance = 0.075

        self._side_length = self.get_parameter("side_length").get_parameter_value().double_value
        self._loops = self.get_parameter("loops").get_parameter_value().integer_value
        self._clockwise = self.get_parameter("clockwise").get_parameter_value().bool_value

        self._drive_speed = 0.05
        self._rotation_speed = 0.1

    def _update_odom(self, msg: Odometry):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y

        quat = msg.pose.pose.orientation
        self._angle = quaternion_to_yaw(quat.x, quat.y, quat.z, quat.w)

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
                self._publisher.publish(Twist())
                break

            msg.angular.z = -self._rotation_speed if error > 0 else self._rotation_speed
            self._publisher.publish(msg)
            time.sleep(0.01)
        self._publisher.publish(Twist())

    def _drive_length(self, meters):
        start_x = self._x
        start_y = self._y

        msg = Twist()
        msg.linear.x = self._drive_speed
        while math.sqrt(pow(start_x - self._x, 2) + pow(start_y - self._y, 2)) < meters:
            self._publisher.publish(msg)
            time.sleep(0.01)

        self._publisher.publish(Twist())

    def run(self):
        self._start_clock = self.get_clock().now().nanoseconds
        if self._clockwise:
            self._rotate_angle(90)
        else:
            self._rotate_angle(180)
        time.sleep(1)

        for k in range(self._loops):
            for i in range(1, 5):
                self._drive_length(self._side_length)

                if self._clockwise:
                    rotation_angle = 90 + 90 * i
                    if rotation_angle > 360:
                        rotation_angle -= 360
                    self._rotate_angle(rotation_angle)
                else:
                    rotation_angle = 180 - 90 * i
                    if rotation_angle < 0:
                        rotation_angle += 360
                    self._rotate_angle(rotation_angle)

            self.get_logger().info(f"Loop {k+1} completed!")

        self.get_logger().info("Finished")



def main():
    rclpy.init()

    node = RectangleDrawer()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin)
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