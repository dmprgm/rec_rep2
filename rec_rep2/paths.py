"""
Shared filesystem/environment locations for rec_rep2.

Single source of truth for paths that used to be hardcoded separately in
recorder.py, gui.py, and scripts/launch_gui.sh (each with its own copy of
`~/ros2_ws/src/rec_rep2/bags`, `/opt/ros/humble/...`, etc). Follows the same
env-var-override, sensible-default pattern already used in
kortex_connection.py for robot connection settings.

Override any of these without editing code:
  REC_REP2_BAGS_DIR   - where recordings are saved and browsed from
  REC_REP2_WS_OVERLAY - workspace install/setup.bash sourced by gui.py's
                        spawned subprocesses
  REC_REP2_ROS_DISTRO - ROS distro name used to build ROS_SETUP; falls back
                        to $ROS_DISTRO (if already sourced in the parent
                        shell) and then to 'jazzy'
"""

import os

BAGS_DIR = os.path.expanduser(
    os.environ.get('REC_REP2_BAGS_DIR', '~/ros2_ws/src/rec_rep2/bags')
)

WS_OVERLAY = os.path.expanduser(
    os.environ.get('REC_REP2_WS_OVERLAY', '~/ros2_ws/install/setup.bash')
)

ROS_DISTRO = os.environ.get(
    'REC_REP2_ROS_DISTRO', os.environ.get('ROS_DISTRO', 'jazzy')
)
ROS_SETUP = f'/opt/ros/{ROS_DISTRO}/setup.bash'
