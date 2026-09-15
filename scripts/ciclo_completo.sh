#!/bin/bash

set -e

ROOT="$HOME/Documentos/Robotica/PARCIAL/Entrega2/ABB-IRB-120---TAP02_RI_KMVM_VYR"

cd "$ROOT"

source /opt/ros/jazzy/setup.bash
source install/setup.bash

export LC_NUMERIC=C

# Cambiar por el planner que gano en 4A
PLANNER="RRTstarkConfigDefault"
echo "4A: HOME -> PRE-PICK"

ros2 launch irb120_cartesian named_move.launch.py \
target:="Pre-pick" \
planner:="$PLANNER"


echo "4B: PRE-PICK -> PICK"

ros2 launch irb120_cartesian cartesian_path.launch.py \
csv:="$ROOT/scripts/waypoints/4B_quintico_ida.csv"


echo "4C: PICK -> PRE-PLACE"

ros2 launch irb120_cartesian named_move.launch.py \
target:="Pre-place" \
planner:="$PLANNER"


echo "4D: PRE-PLACE -> PLACE"

ros2 launch irb120_cartesian cartesian_path.launch.py \
csv:="$ROOT/scripts/waypoints/4D_quintico_ida.csv"


echo "Ciclo terminado"
