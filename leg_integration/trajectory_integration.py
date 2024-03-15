from MotionPlanning.kinematics.kinematics_solvers import KinematicsSolver
from elkapod_driver.driver import ElkapodDriver
import numpy as np
import time
from leg_trajectory import LegTrajectory

if __name__ == "__main__":
    m1 = np.array([0, 0, 0.05])
    a1 = np.array([0.05, 0, 0])
    a2 = np.array([0.09, 0, 0])
    a3 = np.array([0.205, 0, 0])

    kinematics_solver = KinematicsSolver(m1, a1, a2, a3)
    driver = ElkapodDriver()

    leg_nb = 0              # In this example leg number is equivalent to leg sequence number
    movement_time = 10      # Length of the whole leg movement period including swing and stand phases (in seconds)
    swing_percentage = 0.5  # Simulates the tripod gait leg movement

    # All dimensions in meters
    leg_spacing = 0.2
    step_length = 0.1
    base_height = 0.1

    p = np.array([leg_spacing, step_length/2, -base_height])
    trajectory = LegTrajectory(leg_nb, leg_nb, step_length, base_height, movement_time, swing_percentage, p)
    trajectory.generate_symmetrical_trajectory()

    # Move to the start position
    driver.send_one_leg_frame(1, [1, 1, 1], kinematics_solver.inverse(p).tolist())

    # Wait for 2 seconds
    time.sleep(2)

    # Period between next discrete points of the trajectory will be probed and next send to the controller
    send_time = 0.01

    t1 = time.time()
    t3 = time.time()
    while True:
        t2 = time.time()
        if t2 - t1 < movement_time:
            if t2 - t3 > send_time:
                p = trajectory.get_position(t2 - t1, 0)

                q = kinematics_solver.inverse(p)

                angles = [float(q[0]) + 90, float(q[1]) + 90, -float(q[2])]

                driver.send_one_leg_frame(1, [1, 1, 1], angles)
                t3 = time.time()
        else:
            break

    driver.close_conn()



