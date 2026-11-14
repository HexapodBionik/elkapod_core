import math

import numpy as np


def quat_rotate_inverse_np(quat_xyzw: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate world-frame vector v into the body frame.

    Same as isaaclab.utils.math.quat_rotate_inverse, with a body->world quaternion.
    """
    qx, qy, qz, qw = (float(q) for q in quat_xyzw[:4])
    q_vec = np.array([qx, qy, qz], dtype=v.dtype)
    a = v * (2.0 * qw * qw - 1.0)
    b = np.cross(q_vec, v) * (qw * 2.0)
    c = q_vec * (float(np.dot(q_vec, v)) * 2.0)
    return a - b + c


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    """Yaw (rotation about z) of a quaternion, in radians."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
