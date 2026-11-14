#!/usr/bin/env python3
"""
Elkapod RL policy deployment node.

Runs an skrl-trained policy on state from Isaac Sim's ROS2 bridge and publishes
joint position targets: target = default_joint_pos + action_scale * action.

The same node runs both policies. A config with a height_scan section selects the
rough-terrain policy, which additionally observes a height scan sampled from
elevation_mapping_cupy (see elkapod_rl_deploy.elevation_map).
"""

import os

import numpy as np
import rclpy
import torch
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState

from elkapod_rl_deploy.config import PolicyConfig
from elkapod_rl_deploy.elevation_map import ElevationMapScanner
from elkapod_rl_deploy.joint_remap import JointStateMapper
from elkapod_rl_deploy.math_utils import quat_rotate_inverse_np
from elkapod_rl_deploy.observation import (
    GRAVITY_DIR,
    GaitClock,
    Observation,
    ObsTerm,
    RobotState,
)
from elkapod_rl_deploy.policy import load_policy


class PolicyNode(Node):
    def __init__(self):
        super().__init__("elkapod_policy_node")
        config_file, checkpoint_path = self._read_parameters()

        self.cfg = PolicyConfig.from_yaml(config_file)
        self.state = RobotState.initial(self.cfg)
        self.gait = GaitClock(self.cfg)

        self.scanner = None
        if self.cfg.height_scan is not None:
            self.scanner = ElevationMapScanner(self, self.cfg.height_scan)

        self.observation = self._declare_observation()

        device = torch.device("cuda" if self.cfg.use_cuda and torch.cuda.is_available() else "cpu")
        self.policy = load_policy(
            self.cfg,
            checkpoint_path,
            self.observation,
            device,
            scan_shape=self.scanner.shape if self.scanner else None,
        )
        self.get_logger().info(f"Loaded {checkpoint_path} on {device}.")

        self._joint_mapper: JointStateMapper | None = None
        self._startup_steps_remaining = int(self.cfg.startup_seconds * self.cfg.control_rate_hz)
        self._step_count = 0

        self._create_ros_interfaces()
        self.get_logger().info(
            f"Policy node ready: {self.observation.size} observations, "
            f"{self.cfg.num_action_joints} actions, {self.cfg.control_rate_hz} Hz."
        )

    def _read_parameters(self) -> tuple[str, str]:
        # Isaac Sim runs slower than real time, so the control loop must follow /clock.
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

        self.declare_parameter("config_file", "")
        self.declare_parameter("checkpoint_path", "")
        config_file = self.get_parameter("config_file").get_parameter_value().string_value
        checkpoint_path = self.get_parameter("checkpoint_path").get_parameter_value().string_value

        if not os.path.isfile(config_file):
            raise FileNotFoundError(f"config_file not found: '{config_file}'")
        if not os.path.isfile(checkpoint_path):
            raise FileNotFoundError(f"checkpoint_path not found: '{checkpoint_path}'")
        return config_file, checkpoint_path

    def _declare_observation(self) -> Observation:
        """Observation terms, in the same order as the training ObservationsCfg."""
        cfg, state = self.cfg, self.state
        num_joints = cfg.num_obs_joints

        def joint_pos():
            if cfg.joint_pos_relative:
                return state.joint_pos - cfg.default_joint_pos_in_obs
            return state.joint_pos

        terms = [
            ObsTerm("base_lin_vel", 3, lambda: state.base_lin_vel),
            ObsTerm("base_ang_vel", 3, lambda: state.base_ang_vel),
        ]
        if cfg.use_gait_phase:
            terms.append(ObsTerm("gait_phase", self.gait.dim, self.gait.observe))
        terms += [
            ObsTerm("projected_gravity", 3, lambda: state.projected_gravity),
            ObsTerm("joint_pos", num_joints, joint_pos),
            # Default joint velocity is 0, so this is also mdp.joint_vel_rel.
            ObsTerm("joint_vel", num_joints, lambda: state.joint_vel),
            ObsTerm("command", 3, lambda: state.command),
            ObsTerm("last_action", cfg.num_action_joints, lambda: state.last_action),
        ]
        if self.scanner is not None:
            terms.append(ObsTerm("height_scan", self.scanner.size, self.scanner.read))

        if cfg.debug_zero_velocities:
            for term in terms:
                if term.name in ("base_lin_vel", "base_ang_vel", "joint_vel"):
                    term.read = lambda size=term.size: np.zeros(size, dtype=np.float32)
        return Observation(terms)

    def _create_ros_interfaces(self):
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, sensor_qos)
        self.create_subscription(Imu, "/imu", self._imu_cb, sensor_qos)
        self.create_subscription(Odometry, "/odom", self._odom_cb, sensor_qos)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_cb, 10)

        self.cmd_pub = self.create_publisher(JointState, "/joint_command", 10)
        self.timer = self.create_timer(1.0 / self.cfg.control_rate_hz, self._control_step)

    def _joint_state_cb(self, msg: JointState):
        if self._joint_mapper is None:
            self._joint_mapper = JointStateMapper(msg.name, self.cfg.obs_joint_names)
            self._log_joint_mapping(self._joint_mapper)
        self._joint_mapper.apply(
            msg.position, msg.velocity, self.state.joint_pos, self.state.joint_vel
        )

    def _log_joint_mapping(self, mapper: JointStateMapper):
        log = self.get_logger()
        log.info(
            f"{mapper.num_mapped}/{self.cfg.num_obs_joints} observed joints "
            f"found in /joint_states."
        )
        if mapper.unknown:
            log.info(f"Ignoring unknown joints from /joint_states: {mapper.unknown}")
        if mapper.missing:
            log.error(
                f"Joints missing from /joint_states: {mapper.missing}. Their observations "
                f"stay frozen at the default pose, so the policy will misbehave."
            )

    def _imu_cb(self, msg: Imu):
        av = msg.angular_velocity
        self.state.base_ang_vel[:] = (av.x, av.y, av.z)
        q = msg.orientation
        self.state.base_quat_xyzw[:] = (q.x, q.y, q.z, q.w)
        self.state.projected_gravity = quat_rotate_inverse_np(
            self.state.base_quat_xyzw, GRAVITY_DIR
        )

    def _odom_cb(self, msg: Odometry):
        # Isaac Sim publishes odom twist in world frame by default.
        lin = msg.twist.twist.linear
        v = np.array([lin.x, lin.y, lin.z], dtype=np.float32)
        if self.cfg.rotate_odom_to_body:
            v = quat_rotate_inverse_np(self.state.base_quat_xyzw, v)
        self.state.base_lin_vel = v

    def _cmd_vel_cb(self, msg: Twist):
        self.state.command[:] = (msg.linear.x, msg.linear.y, msg.angular.z)

    def _control_step(self):
        if self._joint_mapper is None:
            return
        if self._holding_default_pose():
            self._publish_targets(self.cfg.default_joint_pos)
            return

        obs = self.observation.build()
        action = self.policy.act(obs)
        self.state.last_action = action
        self._publish_targets(self.cfg.default_joint_pos + self.cfg.action_scale * action)

        dump_every = self.cfg.debug_obs_dump_every_n_steps
        if dump_every > 0 and self._step_count % dump_every == 0:
            self.get_logger().info(self._format_observation(obs))
        self._step_count += 1
        self.gait.advance()

    def _holding_default_pose(self) -> bool:
        """True during the startup delay and while the height scanner waits for a map."""
        if self._startup_steps_remaining > 0:
            self._startup_steps_remaining -= 1
            if self._startup_steps_remaining == 0:
                self.get_logger().info("Startup delay complete, engaging policy.")
            return True
        if self.scanner is not None and not self.scanner.ready:
            self.get_logger().info(
                "Waiting for the first elevation map, holding default pose.",
                throttle_duration_sec=2.0,
            )
            return True
        return False

    def _publish_targets(self, positions: np.ndarray):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.cfg.action_joint_names)
        msg.position = positions.tolist()
        self.cmd_pub.publish(msg)

    def _format_observation(self, obs: np.ndarray) -> str:
        np.set_printoptions(precision=4, suppress=True, linewidth=200)
        lines = [f"----- Observation dump (step {self._step_count}) -----"]
        for name, values in self.observation.split(obs).items():
            text = self.scanner.describe(values) if name == "height_scan" else f"({values})"
            lines.append(f"  {name + f'[{values.shape[0]}]':<22} {text}")
        return "\n".join(lines)


def main(args=None):
    rclpy.init(args=args)
    node = PolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
