import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist


qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)

class ElkapodRotTest(Node):
    def __init__(self):
        super().__init__(node_name="elkapod_rot_test")
        self._start_delay = 5       # in seconds
        self._frequency = 20        # in Hz

        self._angular_velocity = 0.1
        self._publishing_time = 2*np.pi / self._angular_velocity


        self._velocity_pub = self.create_publisher(Twist, "/cmd_vel", qos_profile=10)

        self._init_timer = self.create_timer(self._start_delay, self.init, autostart=True)
        self._run_timer = self.create_timer(1/self._frequency, self.run, autostart=False)

        self._start_time = 0
        self._current_time = 0

    def init(self):
        self.get_logger().info("Started publishing commands")
        self._start_time = self.get_clock().now().nanoseconds
        msg = Twist()   
        msg.angular.z = self._angular_velocity
        self._velocity_pub.publish(msg)
        self._run_timer.reset()
        self._init_timer.cancel()

    def run(self):
        self._current_time = self.get_clock().now().nanoseconds


        if self._current_time - self._start_time >= self._publishing_time * 1e9:
            msg = Twist()
            self.get_logger().info("Ended publishing commands")
            self._velocity_pub.publish(msg)
            self._run_timer.cancel()
        else:
            self.get_logger().info(f"Elapsed time from start: {(self._current_time - self._start_time)/1e9:.3f} s")
            

    
    

def main(args=None):
    rclpy.init(args=args)
    node = ElkapodRotTest()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
