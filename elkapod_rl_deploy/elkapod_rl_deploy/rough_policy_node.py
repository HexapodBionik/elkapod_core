#!/usr/bin/env python3
"""
Elkapod rough-terrain RL policy deployment node.

Same as policy_node, but the policy additionally observes a local height scan
that is sampled from elevation_mapping_cupy instead of Isaac Lab's RayCaster.

Observation layout (must match the training ObservationsCfg declaration order):
    [base_lin_vel(3), base_ang_vel(3), gait_phase(2), projected_gravity(3),
     joint_pos(N), joint_vel(N), commands(3), last_action(N_act),
     height_scan(ny*nx)]

Height scan (mirrors isaaclab.envs.mdp.height_scan with a yaw-aligned GridPattern):
    h[k] = base_z - ground_z(base_xy + R(yaw) @ grid[k]) - offset, clipped
    grid ordering "xy": x varies fastest, so the (ny, nx) image has rows = y, cols = x.
"""

import math
from typing import Optional

import numpy as np
import rclpy
import torch
import torch.nn as nn
from grid_map_msgs.msg import GridMap
from rclpy.duration import Duration
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from elkapod_rl_deploy.policy_node import HexapodPolicyNode, RunningStandardScaler


class HeightScanPolicy(nn.Module):
    """skrl GaussianMixin policy: CNN height encoder + MLP over [proprio, features].

    Module indices match skrl's generated containers so the checkpoint loads strictly:
        height_enc_container: conv, act, conv, act, flatten, linear, act
        net_container:        linear, act, ..., linear (action head, no activation)
    """

    def __init__(
        self,
        num_proprio: int,
        scan_shape: tuple,
        convs: list,
        features: int,
        hidden_sizes: list,
        num_actions: int,
        activation: str = "elu",
    ):
        super().__init__()
        act_cls = {"elu": nn.ELU, "relu": nn.ReLU, "tanh": nn.Tanh}[activation.lower()]
        self.num_proprio = num_proprio
        self.scan_shape = tuple(scan_shape)
        self.num_scan = self.scan_shape[0] * self.scan_shape[1]

        enc = []
        in_ch = 1
        for c in convs:
            enc.append(
                nn.Conv2d(
                    in_ch,
                    int(c["out_channels"]),
                    kernel_size=int(c["kernel_size"]),
                    stride=int(c.get("stride", 1)),
                    padding=int(c.get("padding", 0)),
                )
            )
            enc.append(act_cls())
            in_ch = int(c["out_channels"])
        enc.append(nn.Flatten())
        with torch.no_grad():
            flat_dim = nn.Sequential(*enc)(torch.zeros(1, 1, *self.scan_shape)).shape[1]
        enc += [nn.Linear(flat_dim, features), act_cls()]
        self.height_enc_container = nn.Sequential(*enc)

        layers = []
        in_dim = num_proprio + features
        for h in hidden_sizes:
            layers += [nn.Linear(in_dim, h), act_cls()]
            in_dim = h
        layers.append(nn.Linear(in_dim, num_actions))
        self.net_container = nn.Sequential(*layers)

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        proprio = obs[:, : -self.num_scan]
        scan = obs[:, -self.num_scan :].reshape(-1, 1, *self.scan_shape)
        features = self.height_enc_container(scan)
        return self.net_container(torch.cat([proprio, features], dim=-1))


def decode_grid_map_layer(msg: GridMap, layer: str) -> Optional[np.ndarray]:
    """Return the layer as a (rows, cols) array in unwrapped grid_map index order.

    grid_map index (0, 0) is the +x/+y corner; rows go towards -x, cols towards -y.
    """
    if layer not in msg.layers:
        return None
    arr_msg = msg.data[msg.layers.index(layer)]
    dims = arr_msg.layout.dim
    data = np.asarray(arr_msg.data, dtype=np.float32)
    if len(dims) < 2:
        return None
    if dims[0].label == "row_index":
        # Row-major storage.
        grid = data.reshape(dims[0].size, dims[1].size)
    else:
        # Default grid_map storage: column-major (dim[0] = column_index).
        grid = data.reshape(dims[0].size, dims[1].size).T
    # Undo the circular buffer.
    if msg.outer_start_index or msg.inner_start_index:
        grid = np.roll(
            grid, shift=(-msg.outer_start_index, -msg.inner_start_index), axis=(0, 1)
        )
    return grid


def sample_bilinear_nan(
    grid: np.ndarray, rows_f: np.ndarray, cols_f: np.ndarray
) -> np.ndarray:
    """Bilinear lookup at fractional indices, ignoring NaN / out-of-map neighbours.

    Returns NaN where no neighbour is valid.
    """
    n_rows, n_cols = grid.shape
    r0 = np.floor(rows_f).astype(np.int64)
    c0 = np.floor(cols_f).astype(np.int64)
    fr = rows_f - r0
    fc = cols_f - c0

    acc = np.zeros_like(rows_f, dtype=np.float64)
    wsum = np.zeros_like(rows_f, dtype=np.float64)
    for dr, dc, w in (
        (0, 0, (1.0 - fr) * (1.0 - fc)),
        (1, 0, fr * (1.0 - fc)),
        (0, 1, (1.0 - fr) * fc),
        (1, 1, fr * fc),
    ):
        r = r0 + dr
        c = c0 + dc
        inside = (r >= 0) & (r < n_rows) & (c >= 0) & (c < n_cols)
        vals = np.full(rows_f.shape, np.nan)
        vals[inside] = grid[r[inside], c[inside]]
        ok = np.isfinite(vals)
        acc[ok] += w[ok] * vals[ok]
        wsum[ok] += w[ok]

    out = np.full(rows_f.shape, np.nan)
    has = wsum > 1e-9
    out[has] = acc[has] / wsum[has]
    return out


class RoughHexapodPolicyNode(HexapodPolicyNode):
    def __init__(self):
        super().__init__("hexapod_rough_policy_node")

        hs = self.cfg["height_scan"]
        self.scan_layer = str(hs.get("layer", "inpaint"))
        self.scan_base_frame = str(hs.get("base_frame", "body_link"))
        self.map_timeout = float(hs.get("map_timeout_s", 1.0))
        self.require_map_to_start = bool(hs.get("require_map_to_start", True))

        self._map: Optional[np.ndarray] = None
        self._map_frame = ""
        self._map_geom = None  # (center_x, center_y, length_x, length_y, resolution)
        self._map_recv_time: Optional[Time] = None
        self._scan_valid_frac = 0.0
        self._scan_source = "none"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        topic = str(hs.get("grid_map_topic", "/elevation_mapping_node/elevation_map_filter"))
        self.create_subscription(GridMap, topic, self._grid_map_cb, map_qos)
        self.get_logger().info(
            f"Height scan: {self.scan_shape[0]}x{self.scan_shape[1]} grid from "
            f"{topic} [{self.scan_layer}], base_frame={self.scan_base_frame}, "
            f"offset={self.scan_offset}, clip={self.scan_clip}"
        )

    def _setup_height_scan_grid(self):
        hs = self.cfg["height_scan"]
        size = hs.get("size", [1.2, 1.2])
        res = float(hs.get("resolution", 0.05))
        ordering = str(hs.get("ordering", "xy"))
        self.scan_offset = float(hs.get("offset", 0.1887))
        self.scan_clip = tuple(float(v) for v in hs.get("clip", [-1.0, 1.0]))

        # Same construction as isaaclab.sensors.ray_caster.patterns.grid_pattern.
        x = np.arange(-size[0] / 2, size[0] / 2 + 1.0e-9, res)
        y = np.arange(-size[1] / 2, size[1] / 2 + 1.0e-9, res)
        if ordering not in ("xy", "yx"):
            raise ValueError(f"height_scan.ordering must be 'xy' or 'yx', got '{ordering}'")
        grid_x, grid_y = np.meshgrid(x, y, indexing="xy" if ordering == "xy" else "ij")
        self.scan_offsets_xy = np.stack([grid_x.ravel(), grid_y.ravel()], axis=0)
        self.scan_shape = grid_x.shape
        self.num_scan = grid_x.size

    def _compute_num_obs(self) -> int:
        # Called from the base constructor, before RoughHexapodPolicyNode.__init__ continues.
        self._setup_height_scan_grid()
        return super()._compute_num_obs() + self.num_scan

    def _setup_policy(self, ckpt_path: str):
        ckpt = torch.load(ckpt_path, map_location=self.device, weights_only=False)
        self.get_logger().info(f"Loaded checkpoint: {ckpt_path}")
        if "policy" in ckpt and isinstance(ckpt["policy"], dict):
            policy_sd = ckpt["policy"]
        elif "models" in ckpt and "policy" in ckpt["models"]:
            policy_sd = ckpt["models"]["policy"]
        else:
            policy_sd = ckpt

        enc_cfg = self.cfg["height_encoder"]
        self.policy = HeightScanPolicy(
            num_proprio=self.num_obs - self.num_scan,
            scan_shape=self.scan_shape,
            convs=enc_cfg["convs"],
            features=int(enc_cfg["features"]),
            hidden_sizes=list(self.cfg.get("hidden_sizes", [512, 256, 128])),
            num_actions=self.num_actions,
            activation=self.cfg.get("activation", "elu"),
        ).to(self.device)

        clean_sd = {
            k: v
            for k, v in policy_sd.items()
            if k.startswith(("height_enc_container.", "net_container."))
        }
        try:
            self.policy.load_state_dict(clean_sd, strict=True)
        except RuntimeError as e:
            raise RuntimeError(
                f"Checkpoint does not match the height-scan policy built from the YAML "
                f"(num_obs={self.num_obs} = {self.num_obs - self.num_scan} proprio + "
                f"{self.num_scan} scan). Check height_encoder, hidden_sizes and "
                f"height_scan size/resolution.\n{e}"
            ) from e

        self.state_preprocessor = RunningStandardScaler(self.num_obs, device=self.device)
        self._load_state_preprocessor(ckpt, ckpt_path)
        if self.state_preprocessor.enabled and self.state_preprocessor.mean.shape[0] != self.num_obs:
            raise RuntimeError(
                f"State preprocessor size {self.state_preprocessor.mean.shape[0]} "
                f"!= num_obs {self.num_obs}"
            )

    def _grid_map_cb(self, msg: GridMap):
        grid = decode_grid_map_layer(msg, self.scan_layer)
        if grid is None:
            self.get_logger().error(
                f"Layer '{self.scan_layer}' not in GridMap (layers: {list(msg.layers)})",
                throttle_duration_sec=5.0,
            )
            return
        q = msg.info.pose.orientation
        if abs(q.x) > 1e-3 or abs(q.y) > 1e-3 or abs(q.z) > 1e-3:
            self.get_logger().warn(
                "GridMap pose has a non-identity orientation; it is ignored.",
                throttle_duration_sec=10.0,
            )
        if self._map is None:
            self.get_logger().info(
                f"First elevation map received: {grid.shape} cells @ "
                f"{msg.info.resolution} m in '{msg.header.frame_id}'."
            )
        self._map = grid
        self._map_frame = msg.header.frame_id
        self._map_geom = (
            msg.info.pose.position.x,
            msg.info.pose.position.y,
            msg.info.length_x,
            msg.info.length_y,
            msg.info.resolution,
        )
        self._map_recv_time = self.get_clock().now()

    def _ready_to_engage(self) -> bool:
        if self.require_map_to_start and self._map is None:
            self.get_logger().info(
                "Waiting for the first elevation map -- holding default pose.",
                throttle_duration_sec=2.0,
            )
            return False
        return True

    def _compute_height_scan(self) -> np.ndarray:
        # Flat-ground fallback: base at stand height above the ground gives 0.
        flat = np.zeros(self.num_scan, dtype=np.float32)
        self._scan_valid_frac = 0.0

        if self._map is None:
            self._scan_source = "no map"
            return flat
        age = (self.get_clock().now() - self._map_recv_time).nanoseconds * 1e-9
        if age > self.map_timeout:
            self._scan_source = f"stale map ({age:.2f}s)"
            self.get_logger().warn(
                f"Elevation map is {age:.2f}s old (> {self.map_timeout}s) -- "
                f"feeding flat height scan.",
                throttle_duration_sec=2.0,
            )
            return flat
        try:
            tf = self.tf_buffer.lookup_transform(
                self._map_frame, self.scan_base_frame, Time(), timeout=Duration(seconds=0.0)
            )
        except TransformException as e:
            self._scan_source = "no tf"
            self.get_logger().warn(
                f"TF {self._map_frame} -> {self.scan_base_frame} unavailable ({e}) -- "
                f"feeding flat height scan.",
                throttle_duration_sec=2.0,
            )
            return flat

        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        ox, oy = self.scan_offsets_xy
        px = t.x + c * ox - s * oy
        py = t.y + s * ox + c * oy

        cx, cy, lx, ly, res = self._map_geom
        rows_f = (cx + 0.5 * lx - 0.5 * res - px) / res
        cols_f = (cy + 0.5 * ly - 0.5 * res - py) / res
        ground_z = sample_bilinear_nan(self._map, rows_f, cols_f)

        valid = np.isfinite(ground_z)
        self._scan_valid_frac = float(valid.mean())
        self._scan_source = f"map ({age:.2f}s)"
        heights = np.where(valid, t.z - ground_z - self.scan_offset, 0.0)
        return np.clip(heights, *self.scan_clip).astype(np.float32)

    def _build_observation(self) -> np.ndarray:
        return np.concatenate([super()._build_observation(), self._compute_height_scan()])

    def _format_observation(self, obs_np: np.ndarray) -> str:
        scan = obs_np[-self.num_scan :]
        return (
            super()._format_observation(obs_np)
            + f"\n  height_scan[{self.num_scan}] min={scan.min():.3f} max={scan.max():.3f} "
            f"mean={scan.mean():.3f} valid={self._scan_valid_frac:.0%} source={self._scan_source}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = RoughHexapodPolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
