import numpy as np
from MotionPlanning.kinematics.kinematics_solvers import KinematicsSolver
from elkapod_driver.driver import ElkapodDriver
from general.config_load import load_parameters


if __name__ == "__main__":
    m1, a1, a2, a3 = load_parameters("leg_integration/config/leg_configuration.yaml")

    kinematics_solver = KinematicsSolver(m1, a1, a2, a3)
    driver = ElkapodDriver()
    while True:
        try:
            coordinates = input("Please enter the coordinates (x, y, z) in meters separated by one space: ").split()
            x, y, z = float(coordinates[0]), float(coordinates[1]), float(coordinates[2])
            q = kinematics_solver.inverse(np.array([x, y, z]))
            angles = [float(q[0]) + 90, float(q[1]) + 90, -float(q[2])]
            driver.send_one_leg_frame(1, [1, 1, 1], angles)
        except KeyboardInterrupt:
            break

    driver.close_conn()



