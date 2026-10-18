# elkapod_rl_deploy

ROS2 node that loads an skrl-trained policy (from Isaac Lab) and controls the Elkapod robot simulated in Isaac Sim via the ROS2 bridge Action Graph.

## Policies

| `terrain` | Executable          | Config               | Checkpoint       | Extra inputs |
|-----------|---------------------|----------------------|------------------|--------------|
| `flat`    | `policy_node`       | `hexapod.yaml`       | `best_agent.pt`  | -            |
| `rough`   | `rough_policy_node` | `hexapod_rough.yaml` | `rough_agent.pt` | elevation map, TF `odom -> body_link` |

```bash
ros2 launch elkapod_rl_deploy hexapod_policy.launch.py terrain:=rough
# or, together with robot_state_publisher:
ros2 launch elkapod_bringup isaac.launch.py terrain:=rough
```

`config_file` / `checkpoint_path` launch arguments override the per-terrain defaults.

### Rough-terrain height scan

The rough policy (`hexapod_rough_v5`, 703 obs) uses a per-leg gait clock (`gait_clock: per_leg`:
`[sin x6, cos x6]` of each leg's phase, offsets `gait_phase_offsets`, as `mdp.gait_leg_clock`) instead of
the flat policy's single `[sin, cos]` (`gait_clock: global`), and appends a 25x25 height scan
(1.2 m x 1.2 m @ 0.05 m).
In training it comes from a yaw-aligned Isaac Lab `RayCaster` on `body_link`; at deployment it is
sampled (bilinear) from `elevation_mapping_cupy`, which has to be running:

```bash
ros2 launch elkapod_elevation_mapping elevation_mapping.launch.py
```

Each cell is `body_link_z - ground_z - 0.1887` (stand height), clipped to `[-1, 1]`, so flat ground
reads ~0. Grid ordering is `xy` (x varies fastest). The node holds the default pose until the first
map arrives; afterwards, a stale map (`map_timeout_s`), a missing TF or NaN / off-map cells fall back
to flat ground (0). Topic, layer and frame are set in the `height_scan` section of `hexapod_rough.yaml`.
