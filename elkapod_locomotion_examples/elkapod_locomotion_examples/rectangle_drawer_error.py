import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from scipy.spatial.transform import Rotation as R
import threading
import time
import math
import numpy as np
import os


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
        super().__init__("RectangleDrawer")
        self.declare_parameter("side_length", value=1.0)
        self.declare_parameter("loops", value=1)
        self.declare_parameter("clockwise", value=False)
        self.declare_parameter("export_path", value="")

        self._odom_subscriber = self.create_subscription(Odometry, "/leg_odom", self._update_odom, 10)
        self._true_position_subscriber = self.create_subscription(Odometry, "/ground_truth_odom", self._update_true_odom, 10)
        self._publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        self.create_timer(10, self._update_error)
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
        self._export_path = self.get_parameter("export_path").get_parameter_value().string_value

        self._drive_speed = 0.2
        self._rotation_speed = 0.3

        self._temporary_orientation_errors = []
        self._temporary_position_errors = []

        self._averaged_orientation_errors = []
        self._averaged_position_errors = []

        self._loop_orientation_errors = []
        self._loop_position_errors = []

        self._cumulative_orientation_error = 0
        self._cumulative_position_error = 0

        self._average_errors_length = 100

    def generate_report(self):
        report = "\n===============================================\n"
        report += "Temporary errors\n"
        report += "===============================================\n"
        report += f"{'Second':<10} {'Position':<15} {'Orientation':<15}\n"
        report += "---------  --------------  ----------------\n"
        for orientation, position in zip(self._averaged_orientation_errors, self._averaged_position_errors):
            second = (orientation[0] - self._start_clock) / 1e9
            orientation_error = orientation[1]
            position_error = position[1]
            report += f"{second:<9.2f} {position_error:<15.8f} {orientation_error:<15.8e}\n"

        report += "\n===============================================\n"
        report += "Cumulative errors\n"
        report += "===============================================\n"
        report += f"{'Loop':<6} {'Position':<15} {'Orientation':<15}\n"
        report += "-----  --------------  ----------------\n"
        for loop, errors in enumerate(zip(self._loop_orientation_errors, self._loop_position_errors), 1):
            orientation, position = errors
            report += f"{loop:<6} {position:<15.8f} {orientation:<15.8f}\n"

        return report

    def _calculate_loop_cumulative_error(self):
        self._loop_orientation_errors.append(self._cumulative_orientation_error)
        self._loop_position_errors.append(self._cumulative_position_error)

        self._cumulative_orientation_error = 0
        self._cumulative_position_error = 0

    def _update_error(self):
        length = len(self._temporary_position_errors)

        if length > self._average_errors_length:
            # Compute position error
            position_error = sum(self._temporary_position_errors[:self._average_errors_length]) / length

            # Compute orientation error
            orient_error = sum(self._temporary_orientation_errors[:self._average_errors_length]) / length

            self._temporary_orientation_errors = self._temporary_orientation_errors[self._average_errors_length:]
            self._temporary_position_errors = self._temporary_position_errors[self._average_errors_length:]

            current_time = self.get_clock().now().nanoseconds
            self._averaged_position_errors.append([current_time, position_error])
            self._averaged_orientation_errors.append([current_time, orient_error])

            self._cumulative_orientation_error += orient_error
            self._cumulative_position_error += position_error

            self.get_logger().info(f"Temporary position error: {position_error:e}\torientation error: {orient_error:e}")

    def _update_true_odom(self, msg: Odometry):
        self._true_x = msg.pose.pose.position.x
        self._true_y = msg.pose.pose.position.y

        quat = msg.pose.pose.orientation
        self._true_angle = quaternion_to_yaw(quat.x, quat.y, quat.z, quat.w)

    def _update_odom(self, msg: Odometry):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y

        quat = msg.pose.pose.orientation
        self._angle = quaternion_to_yaw(quat.x, quat.y, quat.z, quat.w)

        orientation_error = self._angle - self._true_angle
        if orientation_error > 180:
            orientation_error -= 360
        elif orientation_error < -180:
            orientation_error += 360

        self._temporary_orientation_errors.append(pow(orientation_error, 2))
        self._temporary_position_errors.append(pow(self._x - self._true_x, 2) + pow(self._y - self._true_y, 2))

    def _rotate_angle(self, deg):
        msg = Twist()

        while True:
            error = deg - self._angle
            if error > 180:
                error -= 360
            elif error < -180:
                error += 360

            if abs(error) < self._tolerance:
                self._publisher.publish(Twist())
                break

            msg.angular.z = self._rotation_speed if error > 0 else -self._rotation_speed
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
            self._calculate_loop_cumulative_error()

        self.get_logger().info("Finished")
        self.get_logger().info(self.generate_report())

        if self._export_path != "":
            np.savetxt(os.path.join(self._export_path,
                                    f"position_errors_loops={self._loops}_side_length={self._side_length}.csv"),
                       np.array(self._averaged_position_errors), delimiter=",")

            np.savetxt(os.path.join(self._export_path,
                                    f"orientation_errors_loops={self._loops}_side_length={self._side_length}.csv"),
                       np.array(self._averaged_orientation_errors), delimiter=",")

            np.savetxt(os.path.join(self._export_path,
                                    f"loop_orientation_errors_loops={self._loops}_side_length={self._side_length}.csv"),
                       np.array(self._loop_orientation_errors), delimiter=",")

            np.savetxt(os.path.join(self._export_path,
                                    f"loop_position_errors_loops={self._loops}_side_length={self._side_length}.csv"),
                       np.array(self._loop_position_errors), delimiter=",")


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
