# elkapod_rl_deploy

ROS2 node that loads an skrl-trained policy (from Isaac Lab) and controls the Elkapod robot simulated in Isaac Sim via the ROS2 bridge Action Graph.

## Policies

| `terrain` | Config               | Checkpoint       | Extra inputs |
|-----------|----------------------|------------------|--------------|
| `flat`    | `hexapod.yaml`       | `best_agent.pt`  | -            |
| `rough`   | `hexapod_rough.yaml` | `rough_agent.pt` | elevation map, TF `odom -> body_link` |

```bash
ros2 launch elkapod_rl_deploy elkapod_policy.launch.py terrain:=rough
# or, together with robot_state_publisher:
ros2 launch elkapod_bringup isaac.launch.py terrain:=rough
```

`config_file` / `checkpoint_path` launch arguments override the per-terrain defaults.

### Rough-terrain height scan

The rough policy (`hexapod_rough_v5`, 703 obs) uses a per-leg gait clock (`gait_clock: per_leg`:
`[sin x6, cos x6]` of each leg's phase, offsets `gait_phase_offsets`, as `mdp.gait_leg_clock`) instead of
the flat policy's single `[sin, cos]` (`gait_clock: global`), and appends a 25x25 height scan
(1.2 m x 1.2 m @ 0.05 m).
In training it comes from a yaw-aligned Isaac Lab `RayCaster` on `body_link`. At deployment it is
sampled (bilinear) from `elevation_mapping_cupy`, which has to be running:

```bash
ros2 launch elkapod_elevation_mapping elevation_mapping.launch.py
```

Each cell is `body_link_z - ground_z - 0.1887` (stand height), clipped to `[-1, 1]`, so flat ground
reads ~0. Grid ordering is `xy` (x varies fastest). The node holds the default pose until the first
map arrives. After that, a stale map (`map_timeout_s`), a missing TF or NaN / off-map cells fall back
to flat ground (0). Topic, layer and frame are set in the `height_scan` section of `hexapod_rough.yaml`.

## Code layout

| Module | Content |
|--------|---------|
| `policy_node.py` | The ROS node: subscriptions, observation terms, control loop |
| `config.py` | YAML config as dataclasses, whose field defaults are the only defaults |
| `observation.py` | `RobotState`, `GaitClock` and the ordered observation terms |
| `policy.py` | skrl networks, state preprocessor and checkpoint loading |
| `elevation_map.py` | Height scan from `elevation_mapping_cupy` (GridMap subscription, TF) |
| `height_scan.py` | Height scan sampling math, no ROS |
| `joint_remap.py` | `/joint_states` order to observation joint order |
| `math_utils.py` | Quaternion helpers |

Diagrams of the module dependencies, data flow, startup, control step and classes are in
[docs/architecture.md](docs/architecture.md).

Both terrains run the same `policy_node` executable. A `height_scan` section in the config selects
the rough-terrain policy. The control rate is `control_rate_hz` in the YAML and must equal
`1 / (sim.dt * decimation)` from training.
