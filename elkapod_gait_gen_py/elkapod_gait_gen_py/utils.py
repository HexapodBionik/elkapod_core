import numpy as np

def min_max_norm(val, min_val, max_val, new_min, new_max):
    return (val - min_val) * (new_max - new_min) / (max_val - min_val) + new_min


def rotZ(theta: float) -> np.array:
    rotZ = np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
    ])
    return rotZ

def translate(x: float, y: float, z: float) -> np.array:
    return np.array([x, y, z])
