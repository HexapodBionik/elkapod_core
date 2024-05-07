import numpy as np

class TrajParams:
    def __init__(self):
        self.leg_spacing = 0.6
        self.height = 0.1
        self.vdir = np.pi/2
        self.vval = 0.
        self.omega = 0.
        self.yaw = 0.
        self.pitch = 0.
        self.roll = 0.
        self.step_height = 0.
        self.corpus_position = np.array([0., 0.])

        self.cycle_time = 6.
        self.supportive_legs = [True for _ in range(6)]
        self.gait = '3POINT'
