from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from elkapod_rl_deploy.config import PolicyConfig

GRAVITY_DIR = np.array([0.0, 0.0, -1.0], dtype=np.float32)


@dataclass
class RobotState:
    """Latest sensor readings and command, as fed to the policy."""

    base_lin_vel: np.ndarray
    base_ang_vel: np.ndarray
    projected_gravity: np.ndarray
    # Body->world orientation from /imu, used to rotate /odom twist.
    base_quat_xyzw: np.ndarray
    joint_pos: np.ndarray
    joint_vel: np.ndarray
    command: np.ndarray
    last_action: np.ndarray

    @classmethod
    def initial(cls, cfg: PolicyConfig) -> "RobotState":
        return cls(
            base_lin_vel=np.zeros(3, dtype=np.float32),
            base_ang_vel=np.zeros(3, dtype=np.float32),
            projected_gravity=GRAVITY_DIR.copy(),
            base_quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32),
            joint_pos=cfg.default_joint_pos_in_obs.copy(),
            joint_vel=np.zeros(cfg.num_obs_joints, dtype=np.float32),
            command=np.zeros(3, dtype=np.float32),
            last_action=np.zeros(cfg.num_action_joints, dtype=np.float32),
        )


class GaitClock:
    """Open-loop gait clock, driven by the control step counter as in training."""

    def __init__(self, cfg: PolicyConfig):
        self.mode = cfg.gait_clock
        self.freq = cfg.gait_freq
        self.step_dt = cfg.gait_step_dt
        self.phase_offsets = cfg.gait_phase_offsets
        self.step = 0

    @property
    def dim(self) -> int:
        return 2 * len(self.phase_offsets) if self.mode == "per_leg" else 2

    def observe(self) -> np.ndarray:
        phase = (self.step * self.step_dt * self.freq) % 1.0
        if self.mode == "per_leg":
            ang = 2.0 * np.pi * ((phase + self.phase_offsets) % 1.0)
            return np.concatenate([np.sin(ang), np.cos(ang)]).astype(np.float32)
        ang = 2.0 * np.pi * phase
        return np.array([np.sin(ang), np.cos(ang)], dtype=np.float32)

    def advance(self):
        self.step += 1


@dataclass
class ObsTerm:
    """One named slice of the observation and the function that produces it."""

    name: str
    size: int
    read: Callable[[], np.ndarray]


class Observation:
    """Observation terms in the order of the training ObservationsCfg."""

    def __init__(self, terms: list[ObsTerm]):
        self.terms = terms
        self.size = sum(term.size for term in terms)

    def build(self) -> np.ndarray:
        """Read every term and concatenate them into the flat policy input."""
        values = []
        for term in self.terms:
            value = np.asarray(term.read(), dtype=np.float32)
            if value.shape != (term.size,):
                raise RuntimeError(
                    f"Observation term '{term.name}' has shape {value.shape}, "
                    f"expected ({term.size},)"
                )
            values.append(value)
        return np.concatenate(values)

    def split(self, obs: np.ndarray) -> dict[str, np.ndarray]:
        """Slice a flat observation back into its named terms."""
        out = {}
        offset = 0
        for term in self.terms:
            out[term.name] = obs[offset:offset + term.size]
            offset += term.size
        return out

    def size_table(self) -> str:
        lines = [f"  {term.name:<18} {term.size:>4}" for term in self.terms]
        lines.append("  " + "-" * 23)
        lines.append(f"  {'TOTAL':<18} {self.size:>4}")
        return "\n".join(lines)
