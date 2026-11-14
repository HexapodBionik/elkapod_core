"""Height scan sampled from an elevation map

Mirrors isaaclab.envs.mdp.height_scan with a yaw-aligned GridPattern:
    h[k] = base_z - ground_z(base_xy + R(yaw) @ grid[k]) - offset, clipped
    grid ordering "xy": x varies fastest, so the (ny, nx) image has rows = y, cols = x.
"""

import math
from dataclasses import dataclass
from typing import NamedTuple

import numpy as np

from elkapod_rl_deploy.config import HeightScanConfig


class MapGeometry(NamedTuple):
    center_x: float
    center_y: float
    length_x: float
    length_y: float
    resolution: float


@dataclass
class HeightScanGrid:
    """Ray pattern of the height_scanner RayCasterCfg and the height_scan ObsTerm."""

    offsets_xy: np.ndarray  # (2, num_rays) ray offsets in the yaw-aligned base frame
    shape: tuple            # (ny, nx) for "xy" ordering
    offset: float           # ELKAPOD_STAND_HEIGHT
    clip: tuple

    @property
    def size(self) -> int:
        return self.offsets_xy.shape[1]

    @classmethod
    def from_config(cls, cfg: HeightScanConfig) -> "HeightScanGrid":
        # Same construction as isaaclab.sensors.ray_caster.patterns.grid_pattern.
        x = np.arange(-cfg.size[0] / 2, cfg.size[0] / 2 + 1.0e-9, cfg.resolution)
        y = np.arange(-cfg.size[1] / 2, cfg.size[1] / 2 + 1.0e-9, cfg.resolution)
        grid_x, grid_y = np.meshgrid(x, y, indexing="xy" if cfg.ordering == "xy" else "ij")
        return cls(
            offsets_xy=np.stack([grid_x.ravel(), grid_y.ravel()], axis=0),
            shape=grid_x.shape,
            offset=cfg.offset,
            clip=tuple(cfg.clip),
        )


def decode_grid_map_layer(msg, layer: str) -> np.ndarray | None:
    """Return a grid_map_msgs/GridMap layer as a (rows, cols) array in unwrapped index order.

    grid_map index (0, 0) is the +x/+y corner. Rows go towards -x, cols towards -y.
    """
    if layer not in msg.layers:
        return None
    arr_msg = msg.data[msg.layers.index(layer)]
    dims = arr_msg.layout.dim
    data = np.asarray(arr_msg.data, dtype=np.float32)
    if len(dims) < 2:
        return None
    if dims[0].label == "row_index":
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
    """Read grid values at points that fall between cell centres (bilinear interpolation).

    rows_f / cols_f are cell indices with a fractional part, e.g. (12.3, 40.7) lies
    between rows 12-13 and columns 40-41. The result is a weighted mean of those
    4 cells, each weighted by how close the point is to it:

        grid[12, 40] * 0.7 * 0.3 = 0.21      grid[12, 41] * 0.7 * 0.7 = 0.49
        grid[13, 40] * 0.3 * 0.3 = 0.09      grid[13, 41] * 0.3 * 0.7 = 0.21

    so the height changes smoothly as the robot moves instead of jumping cell to cell.
    Cells that are NaN or outside the grid are left out and the remaining weights are
    rescaled to sum to 1. Returns NaN where none of the 4 cells is valid.
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


def sample_height_scan(
    scan: HeightScanGrid,
    elevation: np.ndarray,
    geom: MapGeometry,
    base_xyz: tuple[float, float, float],
    base_yaw: float,
) -> tuple[np.ndarray, float]:
    """Height scan around the base, in the map frame. Returns (heights, valid fraction).

    Rays that hit no valid map cell read 0, i.e. flat ground at stand height.
    """
    base_x, base_y, base_z = base_xyz
    c, s = math.cos(base_yaw), math.sin(base_yaw)
    ox, oy = scan.offsets_xy
    px = base_x + c * ox - s * oy
    py = base_y + s * ox + c * oy

    rows_f = (geom.center_x + 0.5 * geom.length_x - 0.5 * geom.resolution - px) / geom.resolution
    cols_f = (geom.center_y + 0.5 * geom.length_y - 0.5 * geom.resolution - py) / geom.resolution
    ground_z = sample_bilinear_nan(elevation, rows_f, cols_f)

    valid = np.isfinite(ground_z)
    heights = np.where(valid, base_z - ground_z - scan.offset, 0.0)
    return np.clip(heights, *scan.clip).astype(np.float32), float(valid.mean())
