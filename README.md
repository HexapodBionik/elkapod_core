# Elkapod control stack
![ROS2 distro](https://img.shields.io/badge/ros--version-jazzy-blue)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![Python Version](https://img.shields.io/badge/python-3.12-g.svg)
[![elkapod](https://github.com/HexapodBionik/Elkapod/actions/workflows/main.yml/badge.svg?branch=elkapod_comm)](https://github.com/HexapodBionik/Elkapod/actions/workflows/main.yml)

## List of packages
- `elkapod_core_bringup`
- `elkapod_description`
- `elkapod_gait_gen_cpp`
- `elkapod_gait_gen_py`
- `elkapod_kinematics`
- `elkapod_locomotion_examples`
- `elkapod_motion_manager`

## Installation and workspace setup
For installation and workspace setup please follow the steps from the [elkapod_stack](https://github.com/HexapodBionik/elkapod_stack) repository.

## How to run (Gazebo Harmonic Sim version)?

After building and sourcing run the following command

```bash
ros2 launch elkapod_core_bringup elkapod_core_bringup_sim.launch.py
```

then when you see that the entire motion stack has successfully started like this
```
[INFO] [elkapod_ik-7]: process started with pid [66700]
[INFO] [elkapod_gait-8]: process started with pid [66701]
[INFO] [elkapod_motion_manager-9]: process started with pid [66702]
[elkapod_ik-7] [INFO] [1752783860.278708856] [elkapod_ik]: Elkapod leg controller initialized!
[elkapod_gait-8] [INFO] [1752783860.282855764] [elkapod_gait]: Elkapod gait generator initialized. Use service to activate.
[elkapod_motion_manager-9] [INFO] [1752783860.284642461] [elkapod_motion_manager]: Elkapod motion manager initialized!
```

from different terminal window (after sourcing) move the Motion Manager into the WALKING state with these steps:
1. Run the command and wait till it completes
```bash
ros2 action send_goal /motion_manager_transition elkapod_msgs/action/MotionManagerTrigger "{transition: 'init'}"
```
2. Then publish the second command for the robot to stand up
```bash
ros2 action send_goal /motion_manager_transition elkapod_msgs/action/MotionManagerTrigger "{transition: 'stand_up'}"
```
3. Finally enable the gait generator
```bash
ros2 service call /motion_manager_walk_enable std_srvs/srv/Trigger 
```

Now you can start using the walking stack by publishing velocity commands of type `geometry_msgs/msg/Twist` at the `/cmd_vel` topic. For example you can use the `teleop_twist_keyboard`

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

