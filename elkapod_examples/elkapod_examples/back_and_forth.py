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
    rotation = R.from_quat([qx, qy, qz, qw])

    _, _, yaw = rotation.as_euler('xyz', degrees=True)

    yaw_normalized = yaw % 360
    return yaw_normalized

class BackAndForth(Node):
    def __init__(self):
        super().__init__("rectangle_drawer")
        self.declare_parameter("distance", value=1.0)

        self._true_position_subscriber = self.create_subscription(Odometry, "/odometry/filtered", self._update_odom, 10)
        self._publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        self._x = 0
        self._y = 0
        self._angle = 0

        self._tolerance = 0.075

        self._distance = self.get_parameter("distance").get_parameter_value().double_value

        self._drive_speed = 0.05
        self._rotation_speed = 0.1

    def stop(self):
        self._publisher.publish(Twist())

    def _update_odom(self, msg: Odometry):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y

        quat = msg.pose.pose.orientation
        self._angle = quaternion_to_yaw(quat.x, quat.y, quat.z, quat.w)

    def _rotate_angle(self, deg):
        msg = Twist()

        while rclpy.ok():
            self.get_logger().info(f"Current angle: {self._angle}")
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
        while math.sqrt(pow(start_x - self._x, 2) + pow(start_y - self._y, 2)) < meters and rclpy.ok():
            self._publisher.publish(msg)
            time.sleep(0.01)

        self._publisher.publish(Twist())

    def run(self):
        self._rotate_angle(0)
        time.sleep(1)
        self.get_logger().info(f"Go forward {self._distance} m")
        self._drive_length(self._distance)
        time.sleep(1)
        self._rotate_angle(180)
        time.sleep(1)
        self.get_logger().info(f"Go back {self._distance} m")
        self._drive_length(self._distance)
        time.sleep(1)
        self._rotate_angle(0)

        self.get_logger().info("Finished")



def main():
    rclpy.init()

    node = BackAndForth()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        spin_thread = threading.Thread(target=executor.spin)
        spin_thread.start()

        time.sleep(2)
        node.run()  
    except KeyboardInterrupt:
        node.stop()
    finally:
        executor.remove_node(node)
        executor.shutdown()
        spin_thread.join()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == '__main__':
    main()