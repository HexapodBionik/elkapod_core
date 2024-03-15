#!/bin/bash

WORKSPACE=$(pwd)
tail=$(basename "$WORKSPACE")

if [ "$tail" != "Elkapod" ]; then
  echo "Wrong working directory! All scripts should be run from the Elkapod directory!"
  exit 1
fi

export PYTHONPATH="$PYTHONPATH:$WORKSPACE"

while [[ $# -gt 0 ]]; do
    key="$1"
    case $key in
        --trajectory)
        python3 leg_integration/trajectory_integration.py
        exit 0
        ;;
        --angle)
        python3 leg_integration/angle_integration.py
        exit 0
        ;;
        --inverse-kinematics)
        python3 leg_integration/inverse_kinematics_integration.py
        exit 0
        ;;
        *)
        echo "Invalid argument: $key"
        exit 1
        ;;
    esac
    shift
done

echo "Please specify either --trajectory, --angle, or --inverse_kinematics flag."
exit 1
