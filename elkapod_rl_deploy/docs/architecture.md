# elkapod_rl_deploy architecture

How the policy node is put together: module imports, startup, the control loop and the observation layout for the flat and rough policies.

## Module dependencies

Arrows point from the importing module to the imported one. Only `policy_node` and `elevation_map` touch ROS. `height_scan`, `observation` and `policy` can be tested without it.

```mermaid
flowchart TD
    launch["launch/hexapod_policy.launch.py"] -. "starts with config_file, checkpoint_path" .-> node

    subgraph ros ["Uses ROS"]
        node["policy_node.py<br/>PolicyNode"]
        emap["elevation_map.py<br/>ElevationMapScanner"]
    end

    subgraph pure ["No ROS"]
        cfg["config.py<br/>PolicyConfig"]
        obs["observation.py<br/>RobotState, GaitClock, Observation"]
        pol["policy.py<br/>networks, scaler, load_policy"]
        hs["height_scan.py<br/>grid, decoding, sampling"]
        jr["joint_remap.py<br/>JointStateMapper"]
        mu["math_utils.py<br/>quaternion helpers"]
    end

    yaml[("config/*.yaml")] -.-> cfg
    pt[("config/*.pt")] -.-> pol

    node --> cfg
    node --> obs
    node --> pol
    node --> emap
    node --> jr
    node --> mu
    emap --> hs
    emap --> mu
    emap --> cfg
    hs --> cfg
    obs --> cfg
    pol --> cfg
    pol --> obs
```

## Runtime data flow

Subscriber callbacks only write into `RobotState` (and the scanner's map). The 50 Hz timer reads everything, runs the network and publishes. The dashed path exists only when the YAML has a `height_scan` section.

```mermaid
flowchart LR
    js["/joint_states"] --> jm["JointStateMapper"]
    imu["/imu"] --> imucb["_imu_cb"]
    odom["/odom"] --> odomcb["_odom_cb"]
    cmd["/cmd_vel"] --> cmdcb["_cmd_vel_cb"]

    jm -- "joint_pos, joint_vel" --> state[("RobotState")]
    imucb -- "base_ang_vel, quat, projected_gravity" --> state
    odomcb -- "base_lin_vel, rotated to body" --> state
    cmdcb -- "command" --> state

    gm["/elevation_map_filter<br/>GridMap"] -.-> scanner["ElevationMapScanner"]
    tf["TF odom to body_link"] -.-> scanner

    state --> ob["Observation.build"]
    gait["GaitClock"] --> ob
    scanner -. "height_scan" .-> ob

    ob -- "obs vector" --> scaler["RunningStandardScaler"]
    scaler --> net["FlatPolicyNet or<br/>HeightScanPolicyNet"]
    net -- "action, clipped" --> tgt["default_joint_pos +<br/>action_scale * action"]
    net -- "last_action" --> state
    tgt --> out["/joint_command"]
```

## Startup sequence

`PolicyNode.__init__` runs top to bottom. Every consistency check happens here, so a wrong config or checkpoint stops the node before it subscribes to anything.

```mermaid
sequenceDiagram
    participant N as PolicyNode
    participant C as PolicyConfig
    participant S as ElevationMapScanner
    participant O as Observation
    participant L as load_policy

    N->>N: read config_file, checkpoint_path, force use_sim_time
    N->>C: from_yaml(config_file)
    C-->>N: cfg, unknown keys raise ValueError
    N->>N: RobotState.initial(cfg), GaitClock(cfg)
    opt cfg.height_scan is set
        N->>S: ElevationMapScanner(node, cfg.height_scan)
        S->>S: subscribe GridMap, start TF listener
    end
    N->>O: _declare_observation() with terms in training order
    N->>L: load_policy(cfg, checkpoint, observation, device, scan_shape)
    L->>L: check policy and state_preprocessor entries
    L->>L: preprocessor size must equal observation.size
    L->>L: build network from YAML, load_state_dict strict
    L-->>N: Policy
    N->>N: create subscriptions, publisher, control timer
```

## Control step

One call of `_control_step` per timer tick. Nothing is published until the first `/joint_states` message arrives. After that, the robot holds the default pose during the startup delay and while the rough policy waits for its first map.

```mermaid
flowchart TD
    tick(["timer tick"]) --> mapped{"first /joint_states<br/>received?"}
    mapped -- no --> skip(["return, publish nothing"])
    mapped -- yes --> startup{"startup delay<br/>remaining?"}
    startup -- yes --> dec["decrement counter"] --> hold["publish default_joint_pos"]
    startup -- no --> ready{"scanner exists and<br/>no map yet?"}
    ready -- yes --> hold
    ready -- no --> build["obs = observation.build()"]
    build --> act["action = policy.act(obs)"]
    act --> last["state.last_action = action"]
    last --> pub["publish default_joint_pos +<br/>action_scale * action"]
    pub --> dump{"step_count divisible by<br/>debug_obs_dump_every_n_steps?"}
    dump -- yes --> log["log observation dump"] --> adv
    dump -- no --> adv["step_count += 1<br/>gait.advance()"]
```