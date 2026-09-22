#!/usr/bin/env python3
"""
Elkapod RL policy deployment node.

Runs an skrl-trained policy on state from Isaac Sim's ROS2 bridge and
publishes joint position targets.

Observation layout (must match the training ObservationsCfg declaration order):
    [base_lin_vel(3), base_ang_vel(3), gait_phase(2), projected_gravity(3),
     joint_pos(N), joint_vel(N), commands(3), last_action(N_act)]

Actions are joint offsets: target = default_joint_pos + action_scale * action.
"""

import os
from typing import Optional

import numpy as np
import rclpy
import torch
import torch.nn as nn
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState


def quat_rotate_inverse_np(quat_xyzw: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate world-frame vector v into the body frame.

    Same as isaaclab.utils.math.quat_rotate_inverse, with a body->world quaternion.
    """
    qx, qy, qz, qw = (
        float(quat_xyzw[0]),
        float(quat_xyzw[1]),
        float(quat_xyzw[2]),
        float(quat_xyzw[3]),
    )
    q_vec = np.array([qx, qy, qz], dtype=v.dtype)
    a = v * (2.0 * qw * qw - 1.0)
    b = np.cross(q_vec, v) * (qw * 2.0)
    c = q_vec * (float(np.dot(q_vec, v)) * 2.0)
    return a - b + c


class PolicyMLP(nn.Module):
    """MLP with an activation after every Linear except the last one."""

    def __init__(self, num_obs: int, layer_sizes, activation: str = "elu"):
        super().__init__()
        act_cls = {"elu": nn.ELU, "relu": nn.ReLU, "tanh": nn.Tanh}[activation.lower()]
        layers = []
        in_dim = num_obs
        for i, h in enumerate(layer_sizes):
            layers.append(nn.Linear(in_dim, h))
            if i < len(layer_sizes) - 1:
                layers.append(act_cls())
            in_dim = h
        self.net = nn.Sequential(*layers)

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        return self.net(obs)


class RunningStandardScaler:
    """Inference-only mirror of skrl's RunningStandardScaler."""

    def __init__(
        self,
        size: int,
        epsilon: float = 1e-8,
        clip_threshold: float = 5.0,
        device: str = "cpu",
    ):
        self.mean = torch.zeros(size, device=device)
        self.var = torch.ones(size, device=device)
        self.epsilon = epsilon
        self.clip_threshold = clip_threshold
        self.enabled = False

    def load_from_state_dict(self, sd: dict) -> bool:
        for mkey, vkey in [
            ("running_mean", "running_variance"),
            ("mean", "var"),
            ("_running_mean", "_running_variance"),
        ]:
            if mkey in sd and vkey in sd:
                self.mean = sd[mkey].float().to(self.mean.device)
                self.var = sd[vkey].float().to(self.var.device)
                self.enabled = True
                return True
        return False

    def __call__(self, x: torch.Tensor) -> torch.Tensor:
        if not self.enabled:
            return x
        normalized = (x - self.mean) / (torch.sqrt(self.var) + self.epsilon)
        return torch.clamp(normalized, -self.clip_threshold, self.clip_threshold)


class HexapodPolicyNode(Node):
    def __init__(self):
        super().__init__("hexapod_policy_node")

        # Isaac Sim runs slower than real time, so the control loop must follow /clock.
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.get_logger().info(
            "use_sim_time enforced -> control loop and stamps driven by /clock. "
            "Ensure Isaac Sim publishes /clock (ROS2 Clock / sim-time bridge)."
        )

        self.declare_parameter("config_file", "")
        self.declare_parameter("checkpoint_path", "")
        # 0.0 means "use the value from YAML".
        self.declare_parameter("control_rate_hz", 0.0)

        cfg_path = self.get_parameter("config_file").get_parameter_value().string_value
        ckpt_path = (
            self.get_parameter("checkpoint_path").get_parameter_value().string_value
        )
        rate_override = (
            self.get_parameter("control_rate_hz").get_parameter_value().double_value
        )

        if not cfg_path or not os.path.isfile(cfg_path):
            raise FileNotFoundError(f"config_file not found: {cfg_path}")
        if not ckpt_path or not os.path.isfile(ckpt_path):
            raise FileNotFoundError(f"checkpoint_path not found: {ckpt_path}")

        with open(cfg_path, "r") as f:
            self.cfg = yaml.safe_load(f)

        self.control_rate = (
            rate_override
            if rate_override > 0.0
            else float(self.cfg.get("control_rate_hz", 50.0))
        )

        self.device = torch.device(
            "cuda"
            if torch.cuda.is_available() and self.cfg.get("use_cuda", False)
            else "cpu"
        )
        self.get_logger().info(f"Inference device: {self.device}")

        self.obs_joint_names = self.cfg["obs_joint_names"]
        self.action_joint_names = self.cfg["action_joint_names"]
        self.num_obs_joints = len(self.obs_joint_names)
        self.num_action_joints = len(self.action_joint_names)

        self.default_joint_pos = np.array(
            self.cfg["default_joint_pos"], dtype=np.float32
        )
        assert self.default_joint_pos.shape[0] == self.num_action_joints

        self.action_scale = float(self.cfg.get("action_scale", 0.2))
        self.clip_actions = float(self.cfg.get("clip_actions", 100.0))

        self.joint_pos_relative = bool(self.cfg.get("joint_pos_relative", False))
        self.joint_vel_relative = bool(self.cfg.get("joint_vel_relative", False))
        self.rotate_odom_to_body = bool(self.cfg.get("rotate_odom_to_body", False))
        # Diagnostic only: feeds zero velocity observations to the policy.
        self.debug_zero_velocities = bool(self.cfg.get("debug_zero_velocities", True))
        self.debug_obs_dump = bool(self.cfg.get("debug_obs_dump", True))
        self.debug_obs_dump_every_n_steps = int(
            self.cfg.get("debug_obs_dump_every_n_steps", 0)
        )

        self.use_gait_phase = bool(self.cfg.get("use_gait_phase", True))
        self.gait_freq = float(self.cfg.get("gait_freq", 1.5))
        # Must equal the training env.step_dt (sim.dt * decimation).
        self.gait_step_dt = float(self.cfg.get("gait_step_dt", 1.0 / self.control_rate))
        self._gait_step = 0

        # Obs joints that are not action joints get a default of 0.
        self.default_joint_pos_in_obs = np.zeros(self.num_obs_joints, dtype=np.float32)
        action_name_to_default = dict(
            zip(self.action_joint_names, self.default_joint_pos)
        )
        for i, name in enumerate(self.obs_joint_names):
            if name in action_name_to_default:
                self.default_joint_pos_in_obs[i] = action_name_to_default[name]

        self.gait_phase_dim = 2 if self.use_gait_phase else 0
        self.num_obs = (
            3
            + 3
            + self.gait_phase_dim
            + 3
            + 2 * self.num_obs_joints
            + 3
            + self.num_action_joints
        )
        self.num_actions = self.num_action_joints

        hidden_sizes = list(self.cfg.get("hidden_sizes", [512, 256, 128]))
        # Read real layer sizes from the checkpoint; skrl may or may not save
        # the action head as a separate policy_layer.
        layer_sizes = self._infer_layer_sizes(ckpt_path, hidden_sizes)
        self.get_logger().info(f"Inferred network layer output sizes: {layer_sizes}")

        ckpt_obs_dim = self._infer_checkpoint_obs_dim(ckpt_path)
        if ckpt_obs_dim is not None and ckpt_obs_dim != self.num_obs:
            raise RuntimeError(
                f"Observation size mismatch: checkpoint expects {ckpt_obs_dim}, "
                f"this config builds {self.num_obs}.\n"
                f"  base_lin_vel        3\n"
                f"  base_ang_vel        3\n"
                f"  gait_phase          {self.gait_phase_dim}"
                f"   (use_gait_phase={self.use_gait_phase})\n"
                f"  projected_gravity   3\n"
                f"  joint_pos          {self.num_obs_joints}"
                f"   (len(obs_joint_names))\n"
                f"  joint_vel          {self.num_obs_joints}\n"
                f"  commands            3\n"
                f"  last_action        {self.num_action_joints}"
                f"   (len(action_joint_names))\n"
                f"  -------------------------\n"
                f"  TOTAL              {self.num_obs}\n"
                f"Check obs_joint_names (FIXED joints carry 0 DOF and must NOT "
                f"be listed) and use_gait_phase in the YAML."
            )

        self.policy = PolicyMLP(
            num_obs=self.num_obs,
            layer_sizes=layer_sizes,
            activation=self.cfg.get("activation", "elu"),
        ).to(self.device)

        self.state_preprocessor = RunningStandardScaler(
            self.num_obs, device=self.device
        )
        self._load_skrl_checkpoint(ckpt_path)
        self.policy.eval()

        self.base_lin_vel = np.zeros(3, dtype=np.float32)
        self.base_ang_vel = np.zeros(3, dtype=np.float32)
        self.command = np.zeros(3, dtype=np.float32)
        self.projected_gravity = np.array([0.0, 0.0, -1.0], dtype=np.float32)
        # Body->world orientation from /imu, used to rotate /odom twist.
        self.base_quat_xyzw = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        self.joint_pos = self.default_joint_pos_in_obs.copy()
        self.joint_vel = np.zeros(self.num_obs_joints, dtype=np.float32)
        self.last_action = np.zeros(self.num_action_joints, dtype=np.float32)

        self._joint_state_to_obs_idx: Optional[list] = None
        self._obs_dumped = False
        self._step_count = 0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            JointState, "/joint_states", self._joint_state_cb, sensor_qos
        )
        self.create_subscription(Imu, "/imu", self._imu_cb, sensor_qos)
        self.create_subscription(Odometry, "/odom", self._odom_cb, sensor_qos)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_cb, 10)

        self.cmd_pub = self.create_publisher(JointState, "/joint_command", 10)

        self.timer = self.create_timer(1.0 / self.control_rate, self._control_step)

        self.startup_seconds = float(self.cfg.get("startup_seconds", 2.0))
        self._startup_steps_remaining = int(self.startup_seconds * self.control_rate)

        self.get_logger().info(
            f"Hexapod policy node ready. obs_dim={self.num_obs} "
            f"(obs_joints={self.num_obs_joints}, action_joints={self.num_action_joints}), "
            f"rate={self.control_rate} Hz, "
            f"preprocessor={'on' if self.state_preprocessor.enabled else 'off'}"
        )

    def _infer_layer_sizes(self, ckpt_path: str, hidden_sizes_hint: list) -> list:
        """Output dims of the checkpoint's policy Linear layers, in order."""
        ckpt = torch.load(ckpt_path, map_location="cpu", weights_only=False)
        if (
            isinstance(ckpt, dict)
            and "policy" in ckpt
            and isinstance(ckpt["policy"], dict)
        ):
            sd = ckpt["policy"]
        else:
            sd = ckpt if isinstance(ckpt, dict) else {}

        linears = []
        for k, v in sd.items():
            if not k.endswith(".weight"):
                continue
            if "log_std" in k or "value_layer" in k or "value_container" in k:
                continue
            for prefix in ("net_container.", "net.net.", "net."):
                if k.startswith(prefix):
                    rest = k[len(prefix) :]
                    try:
                        idx = int(rest.split(".")[0])
                        linears.append((("hidden", idx), v.shape[0]))
                    except ValueError:
                        pass
                    break
            else:
                if k.startswith("policy_layer.") or k == "policy_layer.weight":
                    linears.append((("head", 9999), v.shape[0]))

        linears.sort(key=lambda x: (0 if x[0][0] == "hidden" else 1, x[0][1]))
        sizes = [s for _, s in linears]
        if not sizes:
            self.get_logger().warn(
                f"Could not infer layer sizes from checkpoint; "
                f"falling back to hidden_sizes={hidden_sizes_hint} with last "
                f"entry replaced by num_actions={self.num_actions}."
            )
            sizes = list(hidden_sizes_hint[:-1]) + [self.num_actions]
        return sizes

    def _infer_checkpoint_obs_dim(self, ckpt_path: str):
        """Input width of the checkpoint's first Linear layer, or None."""
        ckpt = torch.load(ckpt_path, map_location="cpu", weights_only=False)
        sd = ckpt.get("policy", ckpt) if isinstance(ckpt, dict) else {}
        if not isinstance(sd, dict):
            return None
        best = None
        for k, v in sd.items():
            if not k.endswith(".weight") or getattr(v, "ndim", 0) != 2:
                continue
            if "log_std" in k or "value_layer" in k or "value_container" in k:
                continue
            for prefix in ("net_container.", "net.net.", "net."):
                if k.startswith(prefix):
                    try:
                        idx = int(k[len(prefix) :].split(".")[0])
                    except ValueError:
                        continue
                    if best is None or idx < best[0]:
                        best = (idx, int(v.shape[1]))
                    break
        return None if best is None else best[1]

    def _load_skrl_checkpoint(self, path: str):
        ckpt = torch.load(path, map_location=self.device, weights_only=False)
        self.get_logger().info(f"Loaded checkpoint: {path}")

        if "policy" in ckpt and isinstance(ckpt["policy"], dict):
            policy_sd = ckpt["policy"]
        elif "models" in ckpt and "policy" in ckpt["models"]:
            policy_sd = ckpt["models"]["policy"]
        else:
            policy_sd = ckpt

        # Remap skrl Linear layers onto self.policy.net[0], net[2], net[4], ...
        layer_entries = []
        for k, v in policy_sd.items():
            if "log_std" in k or "value_layer" in k or "value_container" in k:
                continue
            if not (k.endswith(".weight") or k.endswith(".bias")):
                continue
            base = k.rsplit(".", 1)[0]
            param_kind = k.rsplit(".", 1)[1]

            group_idx = None
            for prefix in ("net_container.", "net.net.", "net."):
                if base.startswith(prefix):
                    rest = base[len(prefix) :]
                    try:
                        idx = int(rest.split(".")[0])
                        group_idx = ("hidden", idx)
                    except ValueError:
                        pass
                    break
            if group_idx is None and base == "policy_layer":
                group_idx = ("head", 9999)
            if group_idx is None:
                continue

            layer_entries.append((group_idx, param_kind, v))

        grouped = {}
        for gi, kind, v in layer_entries:
            grouped.setdefault(gi, {})[kind] = v

        sorted_keys = sorted(
            grouped.keys(), key=lambda x: (0 if x[0] == "hidden" else 1, x[1])
        )

        clean_sd = {}
        for our_layer_pos, gi in enumerate(sorted_keys):
            seq_idx = 2 * our_layer_pos
            w = grouped[gi].get("weight")
            b = grouped[gi].get("bias")
            if w is not None:
                clean_sd[f"net.{seq_idx}.weight"] = w
            if b is not None:
                clean_sd[f"net.{seq_idx}.bias"] = b

        missing, unexpected = self.policy.load_state_dict(clean_sd, strict=False)
        if missing:
            self.get_logger().warn(f"Missing keys when loading policy: {missing}")
        if unexpected:
            self.get_logger().warn(f"Unexpected keys when loading policy: {unexpected}")

        # skrl stores the preprocessor in a sibling file when store_separately=True.
        preproc_sd = None
        for key in ("state_preprocessor", "_state_preprocessor"):
            if key in ckpt and isinstance(ckpt[key], dict):
                preproc_sd = ckpt[key]
                break

        if preproc_sd is None:
            ckpt_dir = os.path.dirname(path)
            candidates = [
                os.path.join(ckpt_dir, "state_preprocessor.pt"),
                os.path.join(ckpt_dir, "running_state_preprocessor.pt"),
            ]
            for cand in candidates:
                if os.path.isfile(cand):
                    loaded = torch.load(
                        cand, map_location=self.device, weights_only=False
                    )
                    if isinstance(loaded, dict):
                        preproc_sd = loaded
                        self.get_logger().info(
                            f"Loaded sibling state preprocessor file: {cand}"
                        )
                    break

        if preproc_sd is not None:
            if self.state_preprocessor.load_from_state_dict(preproc_sd):
                self.get_logger().info(
                    "Loaded state preprocessor (RunningStandardScaler)."
                )
            else:
                self.get_logger().warn(
                    "Found preprocessor but couldn't parse. "
                    "Continuing WITHOUT normalization (policy will misbehave!)."
                )
        else:
            self.get_logger().warn(
                "No state preprocessor in checkpoint or sibling file "
                "(state_preprocessor.pt) -- policy will misbehave."
            )

    def _joint_state_cb(self, msg: JointState):
        if self._joint_state_to_obs_idx is None:
            obs_name_to_idx = {n: i for i, n in enumerate(self.obs_joint_names)}
            self._joint_state_to_obs_idx = []
            unknown = []
            for incoming_name in msg.name:
                if incoming_name in obs_name_to_idx:
                    self._joint_state_to_obs_idx.append(obs_name_to_idx[incoming_name])
                else:
                    self._joint_state_to_obs_idx.append(-1)
                    unknown.append(incoming_name)
            mapped_obs_idx = {i for i in self._joint_state_to_obs_idx if i >= 0}
            missing_obs = [
                name
                for i, name in enumerate(self.obs_joint_names)
                if i not in mapped_obs_idx
            ]
            n_mapped = len(mapped_obs_idx)
            self.get_logger().info(
                f"Joint remap: {n_mapped}/{self.num_obs_joints} obs joints found in /joint_states."
            )
            if unknown:
                self.get_logger().info(
                    f"  Ignoring unknown joints from /joint_states: {unknown}"
                )
            if missing_obs:
                self.get_logger().error(
                    f"*** {len(missing_obs)} obs joint(s) NOT in /joint_states: {missing_obs}. "
                    f"Their entries in the observation will stay frozen at their init values "
                    f"({{0 for FCP, default for J*}}). This is almost certainly why the policy "
                    f"misbehaves -- the trained network saw these moving."
                )
            self.get_logger().info(
                f"Incoming /joint_states order ({len(msg.name)} joints): {list(msg.name)}"
            )
            self.get_logger().info(
                f"Configured obs_joint_names order ({self.num_obs_joints}): {self.obs_joint_names}"
            )

        for i, target_idx in enumerate(self._joint_state_to_obs_idx):
            if target_idx < 0:
                continue
            if i < len(msg.position):
                self.joint_pos[target_idx] = msg.position[i]
            if i < len(msg.velocity):
                self.joint_vel[target_idx] = msg.velocity[i]

    def _imu_cb(self, msg: Imu):
        # base_ang_vel comes only from /imu (body frame).
        self.base_ang_vel[0] = msg.angular_velocity.x
        self.base_ang_vel[1] = msg.angular_velocity.y
        self.base_ang_vel[2] = msg.angular_velocity.z

        self.base_quat_xyzw[0] = msg.orientation.x
        self.base_quat_xyzw[1] = msg.orientation.y
        self.base_quat_xyzw[2] = msg.orientation.z
        self.base_quat_xyzw[3] = msg.orientation.w

        self.projected_gravity = quat_rotate_inverse_np(
            self.base_quat_xyzw,
            np.array([0.0, 0.0, -1.0], dtype=np.float32),
        )

    def _odom_cb(self, msg: Odometry):
        # Isaac Sim publishes odom twist in world frame by default.
        v = np.array(
            [
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
            ],
            dtype=np.float32,
        )
        if self.rotate_odom_to_body:
            self.base_lin_vel = quat_rotate_inverse_np(self.base_quat_xyzw, v)
        else:
            self.base_lin_vel = v

    def _cmd_vel_cb(self, msg: Twist):
        self.command[0] = msg.linear.x
        self.command[1] = msg.linear.y
        self.command[2] = msg.angular.z

    def _gait_phase_obs(self) -> np.ndarray:
        """Open-loop gait clock [sin, cos], driven by the step counter as in training."""
        phase = (self._gait_step * self.gait_step_dt * self.gait_freq) % 1.0
        ang = 2.0 * np.pi * phase
        return np.array([np.sin(ang), np.cos(ang)], dtype=np.float32)

    def _build_observation(self) -> np.ndarray:
        jp = (
            self.joint_pos - self.default_joint_pos_in_obs
            if self.joint_pos_relative
            else self.joint_pos
        )
        jv = self.joint_vel  # default velocity is 0, so the _rel variant is identical
        blv = self.base_lin_vel
        bav = self.base_ang_vel
        if self.debug_zero_velocities:
            blv = np.zeros(3, dtype=np.float32)
            bav = np.zeros(3, dtype=np.float32)
            jv = np.zeros(self.num_obs_joints, dtype=np.float32)
        # Order must match the training ObservationsCfg.
        terms = [blv, bav]
        if self.use_gait_phase:
            # Not gated on the command: training never saw a [0, 0] clock.
            terms.append(self._gait_phase_obs())
        terms += [self.projected_gravity, jp, jv, self.command, self.last_action]
        return np.concatenate(terms).astype(np.float32)

    def _control_step(self):
        if self._joint_state_to_obs_idx is None:
            return

        # Hold the default pose before engaging the policy.
        if self._startup_steps_remaining > 0:
            self._startup_steps_remaining -= 1
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(self.action_joint_names)
            msg.position = self.default_joint_pos.tolist()
            self.cmd_pub.publish(msg)
            if self._startup_steps_remaining == 0:
                self.get_logger().info("Startup ramp complete -- engaging policy.")
            return

        obs_np = self._build_observation()
        if obs_np.shape[0] != self.num_obs:
            self.get_logger().error(
                f"Observation size mismatch: got {obs_np.shape[0]}, expected {self.num_obs}"
            )
            return

        self._step_count += 1
        should_dump = (self.debug_obs_dump and not self._obs_dumped) or (
            self.debug_obs_dump_every_n_steps > 0
            and self._step_count % self.debug_obs_dump_every_n_steps == 0
        )
        if should_dump:
            self._obs_dumped = True
            np.set_printoptions(precision=4, suppress=True, linewidth=200)
            n_j = self.num_obs_joints
            o = 0
            blv_s = obs_np[o : o + 3]
            o += 3
            bav_s = obs_np[o : o + 3]
            o += 3
            gait_s = None
            if self.use_gait_phase:
                gait_s = obs_np[o : o + 2]
                o += 2
            grav_s = obs_np[o : o + 3]
            o += 3
            jp_s = obs_np[o : o + n_j]
            o += n_j
            jv_s = obs_np[o : o + n_j]
            o += n_j
            cmd_s = obs_np[o : o + 3]
            o += 3
            act_s = obs_np[o:]
            gait_line = (
                f"  gait_phase        ({gait_s})\n" if gait_s is not None else ""
            )
            self.get_logger().info(
                f"----- Observation dump (step {self._step_count}) -----\n"
                f"  base_lin_vel      ({blv_s})\n"
                f"  base_ang_vel      ({bav_s})\n"
                f"{gait_line}"
                f"  projected_gravity ({grav_s})\n"
                f"  joint_pos[{n_j}]    ({jp_s})\n"
                f"  joint_vel[{n_j}]    ({jv_s})\n"
                f"  command           ({cmd_s})\n"
                f"  last_action[{self.num_action_joints}] ({act_s})\n"
                f"  joint_pos_relative={self.joint_pos_relative} "
                f"rotate_odom_to_body={self.rotate_odom_to_body} "
                f"gait=({'on' if self.use_gait_phase else 'off'}, freq={self.gait_freq}, dt={self.gait_step_dt:.4f}) "
                f"preprocessor={'on' if self.state_preprocessor.enabled else 'off'}"
            )

        with torch.no_grad():
            obs_t = torch.from_numpy(obs_np).unsqueeze(0).to(self.device)
            obs_t = self.state_preprocessor(obs_t)
            action_t = self.policy(obs_t)
            action_t = torch.clamp(action_t, -self.clip_actions, self.clip_actions)
            action_np = action_t.squeeze(0).cpu().numpy()

        self.last_action = action_np.copy()
        target_positions = self.default_joint_pos + self.action_scale * action_np

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.action_joint_names)
        msg.position = target_positions.tolist()
        self.cmd_pub.publish(msg)
        self._gait_step += 1


def main(args=None):
    rclpy.init(args=args)
    node = HexapodPolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
