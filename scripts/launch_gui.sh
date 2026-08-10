#!/bin/bash
# Launcher for the rec_rep2 Control Panel.
# Sources the ROS2 base and the workspace overlay before starting the GUI
# node.  Intended to be called by the .desktop entry.
#
# Distro/overlay defaults live in rec_rep2/paths.py (single source of truth
# for the equivalent Python-side constants). This script can't import that
# module -- ROS isn't sourced yet -- so it mirrors the same override knobs:
# set ROS_DISTRO / REC_REP2_WS_OVERLAY in the environment to change them.

source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source "${REC_REP2_WS_OVERLAY:-$HOME/ros2_ws/install/setup.bash}"
exec ros2 run rec_rep2 gui
