import numpy as np
import time
from MotionPlanning.kinematics.kinematics_solvers import KinematicsSolver
from elkapod_driver.driver import ElkapodDriver
from leg_trajectory import LegTrajectory
from general.config_load import load_parameters

if __name__ == "__main__":
    m1, a1, a2, a3 = load_parameters("leg_integration/config/leg_configuration.yaml")

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
    frequency = 100
    send_time = 1/frequency

    discrete_points = np.arange(0, movement_time, send_time)
    positions = [trajectory.get_position(t, 0) for t in discrete_points]
    angles = [kinematics_solver.inverse(p) for p in positions]

    i = 0
    t1 = time.time()
    t3 = time.time()
    while True:
        t2 = time.time()
        if t2 - t1 < movement_time:
            if t2 - t3 > send_time:
                q = angles[i]
                print(q)
                send_angles = [float(q[0]) + 90, float(q[1]) + 90, -float(q[2])]

                driver.send_one_leg_frame(1, [1, 1, 1], send_angles)
                t3 = time.time()
                i += 1
        else:
            break

    driver.close_conn()



