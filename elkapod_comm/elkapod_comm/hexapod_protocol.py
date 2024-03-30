from typing import Tuple
from enum import Enum
from .hexapod_protocol_exceptions import (
    MalformedHexapodInfoFrameError,
    IncompatibleHexapodProtocolVersionError,
    OldHexapodProtocolVersionWarning,
    NewHexapodProtocolVersionWarning
)

__name__ = "Hexapod Protocol"
__doc__ = "Hardware Controller Hexapod Communication Protocol"
__author__ = "Piotr Patek"
__version__ = "2.0.0"


class FrameType(Enum):
    ONE_LEG = 1
    ONE_SERVO = 2
    INFO = 3
    ADC = 4


class ServoOpCodes(Enum):
    START = 1
    STOP = 2
    SET = 3


FRAME_LENGTHS_TRANSMIT = {
    FrameType.ONE_LEG: 14,
    FrameType.ONE_SERVO: 6,
    FrameType.INFO: 2,
    FrameType.ADC: 2
}

FRAME_LENGTHS_RECEIVE = {
    FrameType.INFO: 7,
    FrameType.ADC: 7
}

INFO_FRAME_FIRST_CHECK_BYTE = 0x15
INFO_FRAME_SECOND_CHECK_BYTE = 0x57
INFO_FRAME_LIST = [FRAME_LENGTHS_TRANSMIT[FrameType.INFO], FrameType.INFO.value]


def entry_angle_to_transmit_data(entry: str) -> Tuple[int, int]:
    entry_filtered = entry.replace("\r", "").replace("\n", "")
    try:
        return split_to_integer_and_float_parts(float(entry_filtered))
    except Exception as e:
        print(e)


def split_to_integer_and_float_parts(number: float):
    integer_part = int(number)
    float_part = int((number - integer_part) * 100)

    if integer_part > 255 or float_part > 99:
        raise Exception("Incorrect angle value")
    return integer_part, float_part


def one_servo_frame(
    servo_id: int, servo_op_code: int, angle_int_part: int, angle_float_part: int
):
    one_servo_list = [
        FRAME_LENGTHS_TRANSMIT[FrameType.ONE_SERVO],
        FrameType.ONE_SERVO.value,
        servo_id,
        servo_op_code,
        angle_int_part,
        angle_float_part,
    ]
    return one_servo_list


def one_leg_frame(
    leg_id: int, servo_op_codes: list, angle_int_parts: list, angle_float_parts: list
):
    one_leg_list = [FRAME_LENGTHS_TRANSMIT[FrameType.ONE_LEG], FrameType.ONE_LEG.value]

    for i in range(3):
        one_leg_list.append(leg_id * 10 + (i + 1) * 1)
        one_leg_list.append(servo_op_codes[i])
        one_leg_list.append(angle_int_parts[i])
        one_leg_list.append(angle_float_parts[i])

    return one_leg_list


def check_info_frame(info_data: list):
    try:
        assert FRAME_LENGTHS_RECEIVE[FrameType.INFO] == info_data[0]
    except AssertionError:
        raise MalformedHexapodInfoFrameError

    current_hexapod_version = [int(x) for x in __version__.split(".")]
    got_hexapod_version = [int(info_data[-3]), int(info_data[-2]), int(info_data[-1])]

    if current_hexapod_version[0] != got_hexapod_version[0]:
        raise IncompatibleHexapodProtocolVersionError(__version__, f"{got_hexapod_version[0]}.{got_hexapod_version[1]}.{got_hexapod_version[2]}")
    
    if current_hexapod_version[1] > got_hexapod_version[1]:
        raise NewHexapodProtocolVersionWarning(__version__, f"{got_hexapod_version[0]}.{got_hexapod_version[1]}.{got_hexapod_version[2]}")
    elif current_hexapod_version[1] < got_hexapod_version[1]:
        raise OldHexapodProtocolVersionWarning(__version__, f"{got_hexapod_version[0]}.{got_hexapod_version[1]}.{got_hexapod_version[2]}")
    
