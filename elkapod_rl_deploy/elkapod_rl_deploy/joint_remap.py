"""Map /joint_states entries onto the policy's observation joint order."""

from collections.abc import Sequence

import numpy as np


class JointStateMapper:
    """Index map from an incoming JointState name order to obs_joint_names order."""

    def __init__(self, incoming_names: Sequence[str], obs_joint_names: Sequence[str]):
        obs_name_to_idx = {n: i for i, n in enumerate(obs_joint_names)}
        # -1 marks incoming joints the policy does not observe.
        self.incoming_to_obs = [obs_name_to_idx.get(n, -1) for n in incoming_names]
        self.unknown = [n for n in incoming_names if n not in obs_name_to_idx]
        mapped = {i for i in self.incoming_to_obs if i >= 0}
        self.missing = [n for i, n in enumerate(obs_joint_names) if i not in mapped]
        self.num_mapped = len(mapped)

    def apply(
        self,
        position: Sequence[float],
        velocity: Sequence[float],
        joint_pos: np.ndarray,
        joint_vel: np.ndarray,
    ):
        """Write the message's positions / velocities into the obs-ordered arrays."""
        for i, obs_idx in enumerate(self.incoming_to_obs):
            if obs_idx < 0:
                continue
            if i < len(position):
                joint_pos[obs_idx] = position[i]
            if i < len(velocity):
                joint_vel[obs_idx] = velocity[i]
