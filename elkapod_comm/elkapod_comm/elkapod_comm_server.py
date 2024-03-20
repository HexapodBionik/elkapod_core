import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import spidev
from elkapod_msgs.msg import ElkapodCommLegFrame


class ElkapodCommServer(Node):
    def __init__(self):
        super().__init__("elkapod_comm_server")
        self._elkapod_leg_subscription = self.create_subscription(
            ElkapodCommLegFrame,
            "elkapod_comm_server_leg_frames",
            self.leg_frame_callback,
            10
        )

    def leg_frame_callback(self, msg):
        print(msg.leg_nb)
        print(msg.servo_op_codes)
        print(msg.servo_angles)


def main(args=None):
    rclpy.init(args=args)

    driver = ElkapodCommServer()
    executor = MultiThreadedExecutor(2)

    rclpy.spin(driver, executor)
    driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
