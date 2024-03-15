#!/bin/bash

WORKSPACE=$(pwd)
tail=$(basename "$WORKSPACE")

if [ "$tail" != "Elkapod" ]; then
  echo "Wrong working directory! All scripts should be run from the Elkapod directory!"
  exit 1
fi

if ! echo "$PYTHONPATH" | grep -q "$WORKSPACE"; then
    export PYTHONPATH="$PYTHONPATH:$WORKSPACE"
    echo "$PYTHONPATH:$WORKSPACE"
else
    echo "Set"
fi

python3 ./leg_intergration/integration.py

