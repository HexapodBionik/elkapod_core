"""Inference-only copy of the skrl policy: networks, state preprocessor and checkpoint loading.

Expected checkpoint (skrl agent.save):
    {"policy": <policy state dict>, "state_preprocessor": {"running_mean", "running_variance"}}

The network classes use skrl's module names and indices, so the policy state dict
loads strictly once the training-only entries (log_std, value head) are dropped.
"""

import numpy as np
import torch
import torch.nn as nn

from elkapod_rl_deploy.config import PolicyConfig
from elkapod_rl_deploy.observation import Observation

ACTIVATIONS = {"elu": nn.ELU, "relu": nn.ReLU, "tanh": nn.Tanh}


def _mlp(in_dim: int, hidden_sizes: list, activation: type) -> tuple[list, int]:
    """[Linear, activation] per hidden size. Returns (layers, output width)."""
    layers = []
    for size in hidden_sizes:
        layers += [nn.Linear(in_dim, size), activation()]
        in_dim = size
    return layers, in_dim


class FlatPolicyNet(nn.Module):
    """Proprioceptive policy.

    net_container: linear, act, ..., linear, act
    policy_layer:  linear (action head)
    """

    def __init__(self, num_obs: int, hidden_sizes: list, num_actions: int, activation: type):
        super().__init__()
        layers, width = _mlp(num_obs, hidden_sizes, activation)
        self.net_container = nn.Sequential(*layers)
        self.policy_layer = nn.Linear(width, num_actions)

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        return self.policy_layer(self.net_container(obs))


class HeightScanPolicyNet(nn.Module):
    """Rough-terrain policy: CNN height encoder + MLP over [proprio, height features].

    height_enc_container: conv, act, ..., flatten, linear, act
    net_container:        linear, act, ..., linear (action head)
    The height scan is the last scan_shape[0] * scan_shape[1] entries of the observation.
    """

    def __init__(
        self,
        num_obs: int,
        scan_shape: tuple[int, int],
        convs: list,
        features: int,
        hidden_sizes: list,
        num_actions: int,
        activation: type,
    ):
        super().__init__()
        self.scan_shape = scan_shape
        self.num_scan = scan_shape[0] * scan_shape[1]

        encoder = []
        channels = 1
        for conv in convs:
            encoder += [
                nn.Conv2d(
                    channels,
                    conv["out_channels"],
                    kernel_size=conv["kernel_size"],
                    stride=conv.get("stride", 1),
                    padding=conv.get("padding", 0),
                ),
                activation(),
            ]
            channels = conv["out_channels"]
        encoder.append(nn.Flatten())
        with torch.no_grad():
            flat_dim = nn.Sequential(*encoder)(torch.zeros(1, 1, *scan_shape)).shape[1]
        encoder += [nn.Linear(flat_dim, features), activation()]
        self.height_enc_container = nn.Sequential(*encoder)

        num_proprio = num_obs - self.num_scan
        layers, width = _mlp(num_proprio + features, hidden_sizes, activation)
        layers.append(nn.Linear(width, num_actions))
        self.net_container = nn.Sequential(*layers)

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        proprio = obs[:, :-self.num_scan]
        scan = obs[:, -self.num_scan:].reshape(-1, 1, *self.scan_shape)
        features = self.height_enc_container(scan)
        return self.net_container(torch.cat([proprio, features], dim=-1))


class RunningStandardScaler:
    """skrl's RunningStandardScaler with frozen statistics."""

    def __init__(self, state_dict: dict, device: torch.device,
                 epsilon: float = 1e-8, clip_threshold: float = 5.0):
        self.mean = state_dict["running_mean"].float().to(device)
        self.var = state_dict["running_variance"].float().to(device)
        self.epsilon = epsilon
        self.clip_threshold = clip_threshold

    @property
    def size(self) -> int:
        return self.mean.shape[0]

    def __call__(self, x: torch.Tensor) -> torch.Tensor:
        normalized = (x - self.mean) / (torch.sqrt(self.var) + self.epsilon)
        return torch.clamp(normalized, -self.clip_threshold, self.clip_threshold)


class Policy:
    """Observation in, clipped action out."""

    def __init__(self, net: nn.Module, scaler: RunningStandardScaler,
                 clip_actions: float, device: torch.device):
        self.net = net.to(device).eval()
        self.scaler = scaler
        self.clip_actions = clip_actions
        self.device = device

    def act(self, obs: np.ndarray) -> np.ndarray:
        with torch.no_grad():
            obs_t = torch.from_numpy(obs).unsqueeze(0).to(self.device)
            action_t = self.net(self.scaler(obs_t))
            action_t = torch.clamp(action_t, -self.clip_actions, self.clip_actions)
            return action_t.squeeze(0).cpu().numpy()


def load_policy(
    cfg: PolicyConfig,
    checkpoint_path: str,
    observation: Observation,
    device: torch.device,
    scan_shape: tuple[int, int] | None = None,
) -> Policy:
    """Build the network described by cfg and load its weights from an skrl checkpoint.

    scan_shape is the (ny, nx) height scan image. None selects the proprioceptive policy.
    """
    checkpoint = torch.load(checkpoint_path, map_location=device, weights_only=False)
    for key in ("policy", "state_preprocessor"):
        if key not in checkpoint:
            raise RuntimeError(
                f"{checkpoint_path} has no '{key}' entry (found: {list(checkpoint)}). "
                f"Expected a checkpoint written by skrl agent.save()."
            )

    scaler = RunningStandardScaler(checkpoint["state_preprocessor"], device)
    if scaler.size != observation.size:
        raise RuntimeError(
            f"Observation size mismatch: checkpoint expects {scaler.size}, "
            f"this config builds {observation.size}.\n"
            f"{observation.size_table()}\n"
            f"Check obs_joint_names, use_gait_phase, gait_clock and height_scan in the YAML."
        )

    activation = ACTIVATIONS[cfg.activation.lower()]
    if scan_shape is None:
        net = FlatPolicyNet(
            num_obs=observation.size,
            hidden_sizes=cfg.hidden_sizes,
            num_actions=cfg.num_action_joints,
            activation=activation,
        )
    else:
        net = HeightScanPolicyNet(
            num_obs=observation.size,
            scan_shape=scan_shape,
            convs=cfg.height_encoder.convs,
            features=cfg.height_encoder.features,
            hidden_sizes=cfg.hidden_sizes,
            num_actions=cfg.num_action_joints,
            activation=activation,
        )

    # Keep only the modules the inference network has (drops log_std and the value head).
    module_names = {name for name, _ in net.named_children()}
    weights = {
        key: value
        for key, value in checkpoint["policy"].items()
        if key.split(".")[0] in module_names
    }
    try:
        net.load_state_dict(weights, strict=True)
    except RuntimeError as e:
        raise RuntimeError(
            f"{checkpoint_path} does not match the {type(net).__name__} built from the YAML. "
            f"Check hidden_sizes, activation and height_encoder.\n{e}"
        ) from e

    return Policy(net, scaler, cfg.clip_actions, device)
