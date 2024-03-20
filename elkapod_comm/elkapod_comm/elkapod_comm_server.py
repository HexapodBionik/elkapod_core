import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import spidev
from elkapod_msgs.msg import ElkapodCommLegFrame
from hexapod_protocol import one_servo_frame, one_leg_frame, split_to_integer_and_float_parts, FrameType, frame_lengths


class ElkapodCommServer(Node):
    def __init__(self):
        super().__init__("elkapod_comm_server")
        self.declare_parameter("spi_speed", value=1000000)

        self._elkapod_leg_subscription = self.create_subscription(
            ElkapodCommLegFrame,
            "elkapod_comm_server_leg_frames",
            self._leg_frame_callback,
            10
        )

        self._spi = spidev.SpiDev()
        try:
            self._spi.open(0, 0)
        except Exception:
            self.get_logger().fatal("Cannot open SPI connection to Hexapod Hardware Controller! Aborting...")
            sys.exit(1)
        spi_speed = self.get_parameter("spi_speed").value

        self._spi.max_speed_hz = spi_speed
        self._spi.mode = 0b00

    def _leg_frame_callback(self, msg):
        leg_id = msg.leg_nb
        servo_op_codes = msg.servo_op_codes
        angles = msg.servo_angles

        angle_int_parts = []
        angle_float_parts = []
        for angle in angles:
            int_part, float_part = split_to_integer_and_float_parts(angle)
            angle_int_parts.append(int_part)
            angle_float_parts.append(float_part)

        message2 = one_leg_frame(leg_id, servo_op_codes, angle_int_parts, angle_float_parts)
        self._send_data(frame_lengths[FrameType.ONE_LEG], message2)

    def send_one_leg_frame(self, leg_id: int, servo_op_codes: list, angles: list):
        angle_int_parts = []
        angle_float_parts = []
        for angle in angles:
            int_part, float_part = split_to_integer_and_float_parts(angle)
            angle_int_parts.append(int_part)
            angle_float_parts.append(float_part)

        message2 = one_leg_frame(leg_id, servo_op_codes, angle_int_parts, angle_float_parts)
        self._send_data(frame_lengths[FrameType.ONE_LEG], message2)

    def _send_data(self, nbytes: int, bytes_to_be_send: list):
        # Send how many bytes the MCU should expect to be sent
        self._spi.writebytes2([nbytes])
        self._spi.writebytes2(bytes_to_be_send)


def main(args=None):
    rclpy.init(args=args)

    driver = ElkapodCommServer()
    executor = MultiThreadedExecutor(2)

    rclpy.spin(driver, executor)
    driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
