"""
Round-trip test for rec_rep2/trajectory_io.py's classic_bags-backed
save_waypoints/load_waypoints. Needs a full ROS2 environment w/ classic_bags/
rclpy/sensor_msgs. run this guy under `colcon test`, Not plain pytest.
"""

import shutil
import tempfile

import pytest
from sensor_msgs.msg import JointState

from rec_rep2.trajectory_io import load_waypoints, save_waypoints


def make_joint_state(positions):
    msg = JointState()
    msg.name = [f'joint_{i + 1}' for i in range(len(positions))]
    msg.position = list(positions)
    msg.velocity = [0.0] * len(positions)
    msg.effort = [0.0] * len(positions)
    return msg


@pytest.fixture
def bag_path():
    tmp_dir = tempfile.mkdtemp(prefix='rec_rep2_test_bag_')
    path = tmp_dir + '/test_recording.bag'
    yield path
    shutil.rmtree(tmp_dir, ignore_errors=True)


def test_round_trip_preserves_waypoint_count_and_positions(bag_path):
    waypoints = [
        (0, make_joint_state([0.0, 0.1, 0.2])),
        (50_000_000, make_joint_state([0.01, 0.11, 0.21])),
        (100_000_000, make_joint_state([0.02, 0.12, 0.22])),
    ]
    save_waypoints(bag_path, waypoints)

    loaded = load_waypoints(bag_path)

    assert len(loaded) == len(waypoints)
    for (_, orig_msg), (_, loaded_msg) in zip(waypoints, loaded):
        assert list(loaded_msg.position) == pytest.approx(list(orig_msg.position))


def test_round_trip_preserves_relative_timing(bag_path):
    waypoints = [
        (0, make_joint_state([0.0])),
        (50_000_000, make_joint_state([0.0])),
        (100_000_000, make_joint_state([0.0])),
    ]
    save_waypoints(bag_path, waypoints)

    loaded = load_waypoints(bag_path)
    t_elapsed = [t for t, _ in loaded]

    assert t_elapsed[0] == pytest.approx(0.0, abs=1e-6)
    assert t_elapsed[1] == pytest.approx(0.05, abs=1e-6)
    assert t_elapsed[2] == pytest.approx(0.10, abs=1e-6)
