#!/bin/sh
# run.sh — Run the dual receiver simulator from any location
# Double-click this script or run:  ./run.sh
cd "$(dirname "$0")" && exec ./bin/dual_receiver_sim "$@"
