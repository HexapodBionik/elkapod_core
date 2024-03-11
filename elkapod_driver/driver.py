import spidev
import sys
from .hexapod_protocol import one_servo_frame, one_leg_frame, split_to_integer_and_float_parts, FrameType, frame_lengths
import time


class ElkapodDriver:
    def __init__(self, max_speed=1000000):
        self._spi = spidev.SpiDev()
        self._spi.max_speed_hz = max_speed
        self._spi.mode = 0b00

        try:
            self._spi.open(0, 0)  # Bus 0, Device 0
        except Exception:
            print("[ERROR] Selected SPI interface cannot be opened!")
            sys.exit(1)

    def close_conn(self):
        self._spi.close()
        self._spi = None

    def send_one_servo_frame(self, servo_id: int, servo_op_code: int, angle):
        angle_int_part, angle_float_part = split_to_integer_and_float_parts(angle)

        message2 = one_servo_frame(servo_id, servo_op_code, angle_int_part, angle_float_part)
        self._send_data(frame_lengths[FrameType.ONE_SERVO], message2)

    def send_one_leg_frame(self, leg_id: int, servo_op_codes: list, angles: list):
        angle_int_parts = []
        angle_float_parts = []
        for angle in angles:
            int_part, float_part = split_to_integer_and_float_parts(angle)
            angle_int_parts.append(int_part)
            angle_float_parts.append(float_part)

        message2 = one_leg_frame(leg_id, servo_op_codes, angle_int_parts, angle_float_parts)
        self._send_data(frame_lengths[FrameType.ONE_SERVO], message2)

    def _send_data(self, nbytes: int, bytes: list):
        # Send how many bytes the MCU should expect to be sent
        self._spi.writebytes2(nbytes)
        time.sleep(0.01)
        self._spi.writebytes2(bytes)



