import rclpy
from rclpy.node import Node

from elkapod_msgs.msg import LegPositions
from geometry_msgs.msg import Point
from elkapod_msgs.msg import TrajectoryParameters
import numpy as np

from .traj_params import TrajParams
from .trajectory import traj_shape
from .gait import build_gait


class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__("TrajectoryGenerator")

        self.declare_parameter("frequency", 100)
        self._timer_period = 1./self.get_parameter("frequency").value

        self._publisher = self.create_publisher(
            LegPositions,
            "elkapod_legs_goals",
            10,
        )

        self._subscriber = self.create_subscription(
            TrajectoryParameters,
            "elkapod_trajectory_parameters",
            self._set_new_traj_params,
            10,
        )

        self._time0 = 0.
        self._traj_params = TrajParams()
        self._leg_trajs = [None for _ in range(6)]
        self._update_leg_trajs()

        self._timer = self.create_timer(self._timer_period, self._send_traj)

    def _set_new_traj_params(self, msg):
        self.get_logger().info(f"Received message {msg}.")

        self._traj_params.leg_spacing = msg.leg_spacing
        self._traj_params.height = msg.height
        self._traj_params.vdir = msg.vdir
        self._traj_params.vval = msg.vval
        self._traj_params.omega = msg.omega
        self._traj_params.yaw = msg.yaw
        self._traj_params.pitch = msg.pitch
        self._traj_params.roll = msg.roll
        self._traj_params.step_height = msg.step_height
        self._traj_params.corpus_position = msg.corpus_position

        self._traj_params.cycle_time = msg.cycle_time
        self._traj_params.supportive_legs = msg.supportive_legs
        self._traj_params.gait = msg.gait

        self._update_leg_trajs()

    def _update_leg_trajs(self):
        time_mappings = build_gait(
            self._traj_params.gait,
            self._traj_params.cycle_time,
            self._traj_params.supportive_legs,
        )

        leg_positions = [
            np.array([self._traj_params.leg_spacing/2, 0.15]),
            np.array([self._traj_params.leg_spacing/2, 0.]),
            np.array([self._traj_params.leg_spacing/2, -0.15]),
            np.array([-self._traj_params.leg_spacing/2, 0.15]),
            np.array([-self._traj_params.leg_spacing/2, 0.]),
            np.array([-self._traj_params.leg_spacing/2, -0.15]),
        ]

        for leg_no, (leg_pos, time_mapping) in \
                enumerate(
                        zip(leg_positions,
                            time_mappings),
                        start=1):
            if time_mapping is None:
                self._leg_trajs[leg_no-1] = (lambda t: np.array([0.32, 0., 0.]))
                continue

            shape = traj_shape(
                leg_pos,                        # leg position
                leg_no,                         # leg number
                np.array([0., 0., 1.]),         # normal to plane
                np.array([self._traj_params.vval *           # linear
                          np.cos(self._traj_params.vdir),
                          self._traj_params.vval *
                          np.sin(self._traj_params.vdir)]) * self._traj_params.cycle_time,
                self._traj_params.omega * self._traj_params.cycle_time,   # angular
                self._traj_params.step_height,               # step height
                self._traj_params.height,                    # height
                self._traj_params.corpus_position,           # corpus position
                self._traj_params.yaw,                       # yaw
                self._traj_params.pitch,                     # pitch
                self._traj_params.roll,                      # roll
                0.1                             # margin
            )

            self._leg_trajs[leg_no-1] = (
                lambda t, shape=shape, time_mapping=time_mapping:
                    shape(time_mapping(t))
            )

    def _send_traj(self):
        curr_time = self.get_clock().now()
        msg = LegPositions()
        msg.header.stamp = curr_time.to_msg()

        curr_time = curr_time.nanoseconds/1000000000.
        if curr_time - self._time0 >= self._traj_params.cycle_time:
            self._time0 = curr_time

        msg.leg_positions = [Point(x=0., y=0., z=0.) for _ in range(6)]
        for point, traj in zip(msg.leg_positions, self._leg_trajs):
            traj_point = traj(curr_time - self._time0)
            point.x = float(traj_point[0])
            point.y = float(traj_point[1])
            point.z = float(traj_point[2])
        self._publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    trajectory_generator = TrajectoryGenerator()
    rclpy.spin(trajectory_generator)
    trajectory_generator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
