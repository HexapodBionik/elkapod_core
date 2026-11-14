import numpy as np
from grid_map_msgs.msg import GridMap
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from elkapod_rl_deploy.config import HeightScanConfig
from elkapod_rl_deploy.height_scan import (
    HeightScanGrid,
    MapGeometry,
    decode_grid_map_layer,
    sample_height_scan,
)
from elkapod_rl_deploy.math_utils import yaw_from_quat


class ElevationMapScanner:
    """Keeps the latest elevation map and samples the height scan around the robot base.

    When the map is missing or stale, or the base TF is unavailable, read() returns a
    flat scan (all zeros = flat ground at stand height).
    """

    def __init__(self, node: Node, cfg: HeightScanConfig):
        self.cfg = cfg
        self.grid = HeightScanGrid.from_config(cfg)
        self._node = node
        self._log = node.get_logger()

        self._map: np.ndarray | None = None
        self._map_frame = ""
        self._map_geom: MapGeometry | None = None
        self._map_recv_time: Time | None = None
        # Diagnostics of the last read(), shown in the observation dump.
        self._valid_frac = 0.0
        self._source = "none"

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        node.create_subscription(GridMap, cfg.grid_map_topic, self._grid_map_cb, map_qos)
        self._log.info(
            f"Height scan: {self.shape[0]}x{self.shape[1]} grid from "
            f"{cfg.grid_map_topic} [{cfg.layer}], base_frame={cfg.base_frame}, "
            f"offset={self.grid.offset}, clip={self.grid.clip}"
        )

    @property
    def size(self) -> int:
        return self.grid.size

    @property
    def shape(self) -> tuple[int, int]:
        """(ny, nx) of the scan image."""
        return self.grid.shape

    @property
    def ready(self) -> bool:
        """False while the policy has to wait for the first map."""
        return self._map is not None or not self.cfg.require_map_to_start

    def read(self) -> np.ndarray:
        """Height scan from the latest map, or a flat scan if the map or TF is unusable."""
        flat = np.zeros(self.size, dtype=np.float32)
        self._valid_frac = 0.0

        if self._map is None:
            self._source = "no map"
            return flat

        age = (self._node.get_clock().now() - self._map_recv_time).nanoseconds * 1e-9
        if age > self.cfg.map_timeout_s:
            self._source = f"stale map ({age:.2f}s)"
            self._log.warn(
                f"Elevation map is {age:.2f}s old (> {self.cfg.map_timeout_s}s), "
                f"feeding flat height scan.",
                throttle_duration_sec=2.0,
            )
            return flat

        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, self.cfg.base_frame, Time(), timeout=Duration(seconds=0.0)
            )
        except TransformException as e:
            self._source = "no tf"
            self._log.warn(
                f"TF {self._map_frame} -> {self.cfg.base_frame} unavailable ({e}), "
                f"feeding flat height scan.",
                throttle_duration_sec=2.0,
            )
            return flat

        t = tf.transform.translation
        q = tf.transform.rotation
        heights, self._valid_frac = sample_height_scan(
            self.grid,
            self._map,
            self._map_geom,
            base_xyz=(t.x, t.y, t.z),
            base_yaw=yaw_from_quat(q.x, q.y, q.z, q.w),
        )
        self._source = f"map ({age:.2f}s)"
        return heights

    def describe(self, heights: np.ndarray) -> str:
        """One-line summary of a scan returned by the last read()."""
        return (
            f"min={heights.min():.3f} max={heights.max():.3f} mean={heights.mean():.3f} "
            f"valid={self._valid_frac:.0%} source={self._source}"
        )

    def _grid_map_cb(self, msg: GridMap):
        grid = decode_grid_map_layer(msg, self.cfg.layer)
        if grid is None:
            self._log.error(
                f"Layer '{self.cfg.layer}' not in GridMap (layers: {list(msg.layers)})",
                throttle_duration_sec=5.0,
            )
            return
        q = msg.info.pose.orientation
        if abs(q.x) > 1e-3 or abs(q.y) > 1e-3 or abs(q.z) > 1e-3:
            self._log.warn(
                "GridMap pose has a non-identity orientation, which is ignored.",
                throttle_duration_sec=10.0,
            )
        if self._map is None:
            self._log.info(
                f"First elevation map received: {grid.shape} cells @ "
                f"{msg.info.resolution} m in '{msg.header.frame_id}'."
            )
        self._map = grid
        self._map_frame = msg.header.frame_id
        self._map_geom = MapGeometry(
            center_x=msg.info.pose.position.x,
            center_y=msg.info.pose.position.y,
            length_x=msg.info.length_x,
            length_y=msg.info.length_y,
            resolution=msg.info.resolution,
        )
        self._map_recv_time = self._node.get_clock().now()
