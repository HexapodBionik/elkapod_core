from typing import Callable
import numpy as np
import sys

sys.path.append("./")
from MotionPlanning.kinematics.kinematics_utils import rot_x, rot_y, rot_z


def base_backward_flat_traj_without_omega(pos: np.ndarray,
                                          step_height: float,
                                          linear: np.ndarray,
                                          t: float) -> np.ndarray:
    """! Generates flat backward part of the base trajectory
    Helper function. To be used when angular velocity
    is zero. Generated trajectory is a fragment
    of a straight line.

    @param pos: Base position of the leg
    @param step_height: Step height
    @param linear: Distance travelled in full-cycle
    @param t: Normalized timepoint (0-1)

    @return Point at trajectory
    """
    assert 0. <= t and t <= 1.

    point0 = pos + 0.25*linear
    return np.array([point0[0] - linear[0]*0.5*t,
                     point0[1] - linear[1]*0.5*t,
                     0.])


def base_backward_flat_traj_with_omega(pos: np.ndarray,
                                       step_height: float,
                                       linear: np.ndarray,
                                       angular: np.ndarray,
                                       t: float) -> np.ndarray:
    """! Generates flat backward part of the base trajectory
    Helper function. To be used when angular velocity
    is not zero. Generated trajectory is a fragment
    of a circle.

    @param pos: Base position of the leg
    @param step_height: Step height
    @param linear: Distance travelled in full-cycle
    @param angular: Angle the robot rotates by in full-cycle
    @param t: Normalized timepoint (0-1)

    @return Point at trajectory
    """
    assert 0. <= t and t <= 1.

    # r_c - radius of the circle followed by hexapod
    r_c = np.linalg.norm(linear)/angular

    # vdir - unit vector of direction of the velocity
    if np.linalg.norm(linear) > 0.:
        vdir = linear/np.linalg.norm(linear)
    else:
        vdir = np.array([0., 0.])

    # center - center of the circle followed by hexapod
    odir = np.array([-vdir[1], vdir[0]])
    center = odir*r_c

    # position of the leg relative to the center of
    # the circle followed by hexapod
    rel_pos = pos[:2] - center

    # r - Radius of a circle with same center but
    #     crossing end of the current leg in xy plane.
    r = np.sqrt(rel_pos[0]**2 + rel_pos[1]**2)

    # phi0 - position of the end of the current leg
    #        on such circle
    phi0 = np.arctan2(rel_pos[1], rel_pos[0])

    # phi_p - At timepoint 0 our leg has position phi_p
    phi_p = phi0 + 0.25*angular

    return np.array([r*np.cos(phi_p - angular*0.5*t) + center[0],
                     r*np.sin(phi_p - angular*0.5*t) + center[1],
                     0.])


def base_backward_flat_traj(pos: np.ndarray,
                            step_height: float,
                            linear: np.ndarray,
                            angular: np.ndarray,
                            t: float) -> np.ndarray:
    """! Generates flat backward part of the base trajectory
    Generates flat backward part of the base trajectory using
    appropriate helper function. Which helper function is used
    depends on whether angular shift is to be zero or not.

    @param pos: Base position of the leg
    @param step_height: Step height
    @param linear: Distance travelled in full-cycle
    @param angular: Angle the robot rotates by in full-cycle
    @param t: Normalized timepoint (0-1)

    @return Point at trajectory
    """
    assert 0. <= t and t <= 1.

    if angular == 0.:
        return base_backward_flat_traj_without_omega(pos, step_height,
                                                     linear, t)
    else:
        return base_backward_flat_traj_with_omega(pos, step_height,
                                                  linear,
                                                  angular, t)


def base_forward_flat_traj(pos: np.ndarray,
                           step_height: float,
                           linear: np.ndarray,
                           angular: np.ndarray,
                           t: float) -> np.ndarray:
    """! Generates flat forward part of the base trajectory
    Flat forward move has same shape as backward move and
    the leg moves in the opposite direction.

    @param pos: Base position of the leg
    @param step_height: Step height
    @param linear: Distance travelled in full-cycle
    @param angular: Angle the robot rotates by in full-cycle
    @param t: Normalized timepoint (1-2)

    @return Point at trajectory
    """
    assert 1. <= t and t <= 2.

    return base_backward_flat_traj(pos, step_height,
                                   linear,
                                   angular, 2. - t)


def traj_of_raise(margin: float, x: float) -> np.ndarray:
    """! Generates vertical part of the trajectory
    @param margin: Time period between the backward move and the forward move
    @param x: Normalized time

    @return Point at trajectory of raise
    """
    if x < -1. or x >= 1.:
        return np.array([0., 0., 0.])

    alpha = -1.
    beta = -1. + margin
    gamma = 1. - margin
    delta = 1.

    if x < beta:
        return np.array([0., 0.,
                         (1./(alpha**2 - alpha*beta))*x**2 +
                         (-2./(alpha - beta))*x +
                         alpha/(alpha - beta)])
    elif x < gamma:
        return np.array([0., 0.,
                         (-1./(alpha*beta))*x**2 + 1.])
    else:
        # x >= gamma
        assert x < delta

        return np.array([0., 0.,
                         (1./(alpha**2 - alpha*beta))*x**2 +
                         (2./(alpha - beta))*x +
                         alpha/(alpha - beta)])


def remove_margin(t: float, margin: float) -> float:
    """! Scales time axis such that the margin is removed
    Squeezed generated trajectory so that the backward trajectory
    fits in (margin/2):(1-margin/2) section insted of 0:1 section
    and the forward trajectory fits in (1+margin/2):(2-margin/2)
    section instead of 1:2 section.

    @param t: Input timepoint
    @param margin: Time period between the backward move and the forward move

    @return Output timepoint
    """
    if t < 1.:
        return (t - margin/2)/(1. - margin)

    return (t - 1. - margin/2)/(1. - margin) + 1.


def base_traj(pos: np.ndarray,
              step_height: float,
              linear: np.ndarray,
              angular: np.ndarray,
              margin: float,
              t: float) -> np.ndarray:
    """! Generates the the base trajectory
    @param pos: Base position of the leg
    @param step_height: Step height
    @param linear: Distance travelled in full-cycle
    @param angular: Angle the robot rotates by in full-cycle
    @param t: Normalized timepoint (0-2)

    @return Point at trajectory
    """
    assert 0. <= t and t <= 2.

    if t < margin/2:
        return base_forward_flat_traj(pos, step_height,
                                      linear,
                                      angular, 2.) + \
               step_height*traj_of_raise(margin, 2.*t - 3.)
    elif t >= 2. - margin/2:
        return base_forward_flat_traj(pos, step_height,
                                      linear,
                                      angular, 2.) + \
               step_height*traj_of_raise(margin, 2*t - 3.)
    elif t >= 1. - margin/2 and t < 1. + margin/2:
        return base_backward_flat_traj(pos, step_height,
                                       linear,
                                       angular, 1.) + \
               step_height*traj_of_raise(margin, 2.*t - 3.)
    elif t < 1.:
        return base_backward_flat_traj(pos, step_height,
                                       linear,
                                       angular, remove_margin(t, margin))
    else:
        return base_forward_flat_traj(pos, step_height,
                                      linear,
                                      angular, remove_margin(t, margin)) + \
               step_height*traj_of_raise(margin, 2.*t - 3.)


def add_plane(p: np.ndarray, n: np.ndarray) -> np.ndarray:
    """! Adds plane perpendicular to n to point p
    @param p: Point
    @param n: Vector normal to the plane

    @return New point
    """

    return np.array([p[0], p[1], p[2] + (n[0]*p[0] + n[1]*p[1])/(-n[2])])


def mountpoint_relative(p: np.ndarray,
                        mountpoint_no: int,
                        height: float,
                        yaw: float,
                        pitch: float,
                        roll: float) -> np.ndarray:
    """! Switches the coordinate system to mountpoint relative
    @param p: Point relative to the base point
    @param mountpoint_no: Number of mountpoint on the robot
    @param height: Corpus suspension height
    @param yaw: Corpus rotation in yaw
    @param pitch: Corpus rotation in pitch
    @param roll: Corpus rotation in roll

    @return Point relative to the mountpoint
    """

    # Dimensions of the corpus of the robot
    corpus_dimensions = np.array([0.2, 0.3])

    mountpoint_positions = [
        np.array([corpus_dimensions[0]/2.,
                  corpus_dimensions[1]/2.,
                  height]),
        np.array([corpus_dimensions[0]/2.,
                  0.,
                  height]),
        np.array([corpus_dimensions[0]/2.,
                  -corpus_dimensions[1]/2.,
                  height]),
        np.array([-corpus_dimensions[0]/2.,
                  corpus_dimensions[1]/2.,
                  height]),
        np.array([-corpus_dimensions[0]/2.,
                  0.,
                  height]),
        np.array([-corpus_dimensions[0]/2.,
                  -corpus_dimensions[1]/2.,
                  height]),
    ]

    # Mappings to mountpoint coordinate systems
    mountpoint_cs_mappings = [
        lambda p: p*np.array([1., -1., 1.]),
        lambda p: p*np.array([1., -1., 1.]),
        lambda p: p*np.array([1., -1., 1.]),
        lambda p: p*np.array([-1., -1., 1.]),
        lambda p: p*np.array([-1., -1., 1.]),
        lambda p: p*np.array([-1., -1., 1.]),
    ]

    mount = mountpoint_positions[mountpoint_no-1]
    mapping = mountpoint_cs_mappings[mountpoint_no-1]

    p -= np.array([0., 0., height])
    p = rot_z(-roll) @ p
    p = rot_x(-pitch) @ p
    p = rot_y(-yaw) @ p
    p += np.array([0., 0., height])

    return mapping(p - mount)


def traj_point(leg_pos: np.ndarray,
               mountpoint_no: int,
               n: np.ndarray,
               linear: np.ndarray,
               angular: np.ndarray,
               step_height: float,
               height: float,
               corpus_position: np.ndarray,
               yaw: float,
               pitch: float,
               roll: float,
               margin: float,
               t: float) -> np.ndarray:
    """! Computes coordinates of point at mountpoint-relative trajectory
    @param leg_pos: Base position of the leg
    @param mountpoint_no: Number of the mountpoint the leg is attached to
    @param n: Vector normal to the plane the leg moves on
    @param linear: Linear distance traveled by the leg in one time cycle
    @param angular: Angular distance traveled by the leg in one time cycle
    @param step_height: Step height
    @param height: Corpus suspension height
    @param corpus_position: Position of the corpus on the XY plane
    @param yaw: Corpus rotation in yaw
    @param pitch: Corpus rotation in pitch
    @param roll: Corpus rotation in roll
    @param margin: Time period between the backward move and the forward move
    @param t: Timepoint

    @return Point at trajectory
    """

    # Point at base trajectory
    p = base_traj(
        leg_pos,
        step_height,
        linear,
        angular,
        margin,
        t,
    )

    # Apply plane projection
    p = add_plane(p, n)

    # Adjust corpus position
    p += np.array([
        -corpus_position[0],
        corpus_position[1],
        0.
    ])

    # Translate coordinates
    p = mountpoint_relative(
        p,
        mountpoint_no,
        height,
        yaw,
        pitch,
        roll,
    )

    return p


def traj_shape(leg_pos: np.ndarray,
               mountpoint_no: int,
               n: np.ndarray,
               linear: np.ndarray,
               angular: np.ndarray,
               step_height: float,
               height: float,
               corpus_position: np.ndarray,
               yaw: float,
               pitch: float,
               roll: float,
               margin: float) -> Callable[[float], float]:
    """! Generates lambda returning points at trajectory
    @param leg_pos: Base position of the leg
    @param mountpoint_no: Number of the mountpoint the leg is attached to
    @param n: Vector normal to the plane the leg moves on
    @param linear: Linear distance traveled by the leg in one time cycle
    @param angular: Angular distance traveled by the leg in one time cycle
    @param step_height: Step height
    @param height: Corpus suspension height
    @param corpus_position: Position of the corpus on the XY plane
    @param yaw: Corpus rotation in yaw
    @param pitch: Corpus rotation in pitch
    @param roll: Corpus rotation in roll
    @param margin: Time period between the backward move and the forward move

    @return Trajectory
    """

    return (lambda t: traj_point(
                          leg_pos,
                          mountpoint_no,
                          n,
                          linear,
                          angular,
                          step_height,
                          height,
                          corpus_position,
                          yaw,
                          pitch,
                          roll,
                          margin,
                          t,
                      ))
