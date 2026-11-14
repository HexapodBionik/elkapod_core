from dataclasses import dataclass, field

import numpy as np
import yaml

GAIT_CLOCKS = ("global", "per_leg")
SCAN_ORDERINGS = ("xy", "yx")


@dataclass
class HeightScanConfig:
    """Mirrors the height_scanner RayCasterCfg and the height_scan ObsTerm."""

    size: list = field(default_factory=lambda: [1.2, 1.2])
    resolution: float = 0.05
    ordering: str = "xy"
    # h = base_z - ground_z - offset, clipped. offset is ELKAPOD_STAND_HEIGHT.
    offset: float = 0.1887
    clip: list = field(default_factory=lambda: [-1.0, 1.0])

    grid_map_topic: str = "/elevation_mapping_node/elevation_map_filter"
    layer: str = "inpaint"
    # RayCaster prim in training.
    base_frame: str = "body_link"
    # An older map (or a missing TF) gives a flat height scan.
    map_timeout_s: float = 1.0
    require_map_to_start: bool = True

    def __post_init__(self):
        if self.ordering not in SCAN_ORDERINGS:
            raise ValueError(
                f"height_scan.ordering must be one of {SCAN_ORDERINGS}, got '{self.ordering}'"
            )


@dataclass
class HeightEncoderConfig:
    """CNN height encoder (skrl agent.yaml: models.policy.network)."""

    # Each entry: {out_channels, kernel_size, stride, padding}
    convs: list
    features: int


@dataclass
class PolicyConfig:
    # Joints in Isaac Lab articulation order / JointPositionActionCfg order.
    obs_joint_names: list
    action_joint_names: list
    # ELKAPOD_CFG.init_state.joint_pos, in action_joint_names order.
    default_joint_pos: np.ndarray

    action_scale: float = 0.2
    clip_actions: float = 100.0

    use_cuda: bool = False
    # 1 / (sim.dt * decimation)
    control_rate_hz: float = 50.0
    # Default pose is held this long before the policy is engaged.
    startup_seconds: float = 2.0

    # true if training used mdp.joint_pos_rel
    joint_pos_relative: bool = False
    rotate_odom_to_body: bool = False

    debug_zero_velocities: bool = False
    # Log the observation every N policy steps, starting with the first one. 0 disables it.
    debug_obs_dump_every_n_steps: int = 0

    use_gait_phase: bool = True
    gait_freq: float = 1.5
    # "global": [sin, cos] of one phase (mdp.gait_clock).
    # "per_leg": [sin x6, cos x6] of each leg's phase (mdp.gait_leg_clock).
    gait_clock: str = "global"
    gait_phase_offsets: np.ndarray = field(
        default_factory=lambda: [0.0, 0.5, 0.5, 0.0, 0.0, 0.5]
    )
    # Training env.step_dt (sim.dt * decimation). None means 1 / control_rate_hz.
    gait_step_dt: float | None = None

    # Policy network (skrl agent.yaml: models.policy.network).
    hidden_sizes: list = field(default_factory=lambda: [512, 256, 128])
    activation: str = "elu"

    # Rough-terrain policy only. Both sections must be given together.
    height_scan: HeightScanConfig | None = None
    height_encoder: HeightEncoderConfig | None = None

    def __post_init__(self):
        self.default_joint_pos = np.array(self.default_joint_pos, dtype=np.float32)
        self.gait_phase_offsets = np.array(self.gait_phase_offsets, dtype=np.float32)
        if self.gait_step_dt is None:
            self.gait_step_dt = 1.0 / self.control_rate_hz

        if self.gait_clock not in GAIT_CLOCKS:
            raise ValueError(f"gait_clock must be one of {GAIT_CLOCKS}, got '{self.gait_clock}'")
        if self.default_joint_pos.shape[0] != self.num_action_joints:
            raise ValueError(
                f"default_joint_pos has {self.default_joint_pos.shape[0]} entries, "
                f"expected {self.num_action_joints} (len(action_joint_names))"
            )
        if (self.height_scan is None) != (self.height_encoder is None):
            raise ValueError("height_scan and height_encoder must be given together")

        # Default pose in obs_joint_names order. Obs joints that are not action joints get 0.
        default_by_name = dict(zip(self.action_joint_names, self.default_joint_pos))
        self.default_joint_pos_in_obs = np.array(
            [default_by_name.get(name, 0.0) for name in self.obs_joint_names],
            dtype=np.float32,
        )

    @property
    def num_obs_joints(self) -> int:
        return len(self.obs_joint_names)

    @property
    def num_action_joints(self) -> int:
        return len(self.action_joint_names)

    @classmethod
    def from_yaml(cls, path: str) -> "PolicyConfig":
        with open(path, "r") as f:
            raw = yaml.safe_load(f)
        try:
            if "height_scan" in raw:
                raw["height_scan"] = HeightScanConfig(**raw["height_scan"])
            if "height_encoder" in raw:
                raw["height_encoder"] = HeightEncoderConfig(**raw["height_encoder"])
            return cls(**raw)
        except TypeError as e:
            # Unknown or missing YAML key.
            raise ValueError(f"Invalid config {path}: {e}") from e
