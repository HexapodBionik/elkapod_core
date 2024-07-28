# Elkapod control stack
![ROS2 distro](https://img.shields.io/badge/ros--version-humble-blue)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![Python Version](https://img.shields.io/badge/python-3.10-g.svg)
[![elkapod](https://github.com/HexapodBionik/Elkapod/actions/workflows/main.yml/badge.svg?branch=elkapod_comm)](https://github.com/HexapodBionik/Elkapod/actions/workflows/main.yml)

## List of packages
- `elkapod_comm`
- `elkapod_core_bringup`
- `elkapod_motion`
- `elkapod_trajectory_generator`

## Installation
1. Install [ElkapodAlgorithms](https://github.com/HexapodBionik/ElkapodAlgorithms.git) python package locally on your machine.
2. Create a workspace and clone packages into it
```bash
mkdir -p elkapod_core/src/
git clone https://github.com/HexapodBionik/elkapod_core elkapod_core/src/
```
3. Move into `src/` folder and download all additional packages using [vcstool](http://wiki.ros.org/vcstool)
```bash
cd elkapod_core/src/
vcs import . < repos.yaml
```

## How to run?

After building and sourcing run the following command

```bash
ros2 launch elkapod_core_bringup elkapod_core_bringup.launch.py sim:=False
```

If this repository is used as submodule for [ElkapodSimulation](https://github.com/HexapodBionik/elkapod_simulation) then run

```bash
ros2 launch elkapod_core_bringup elkapod_core_bringup.launch.py sim:=True
```

The default value of `sim` arugment is `True`.

