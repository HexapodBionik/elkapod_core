import numpy as np


class ElkapodLegPathBase:
    def __init__(self, step_length: float, step_height: float):
        self._step_length = step_length
        self._step_height = step_height

        self._x_func_stance = None
        self._y_func_stance = None
        self._z_func_stance = None

        self._x_func_swing = None
        self._y_func_swing = None
        self._z_func_swing = None

    def init(self):
        R = (pow(self._step_height, 2) + pow(self._step_length, 2) / 4) / (2 * self._step_height)
        h = R - self._step_height

        self._x_func_stance = lambda s: self._step_length / 2 - self._step_length * s
        self._y_func_stance = lambda s: 0.0
        self._z_func_stance = lambda s: 0.0

        self._x_func_swing = lambda s: -self._step_length / 2 + self._step_length * s
        self._y_func_swing = lambda s: 0.0
        self._z_func_swing = lambda s: R * np.sin(np.pi * s) - h


    def __call__(self, *args, **kwargs):
        s = args[0]
        phase = args[1]

        if phase:
            p = np.array([self._x_func_swing(s), self._y_func_swing(s), self._z_func_swing(s)])
        else:
            p = np.array([self._x_func_stance(s), self._y_func_stance(s), self._z_func_stance(s)])

        return p