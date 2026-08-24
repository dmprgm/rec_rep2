"""
Shared classic_bags helpers for writing/reading the recorded /joint_states
trajectory. Bags are plain single-topic rosbag2 (sqlite3) bags under the
hood. Classic_bags wraps rosbag2_py, so they can still be inspected with
`ros2 bag info <path>` like any other rosbag2 recording.
"""

import classic_bags

TOPIC = '/joint_states'


def save_waypoints(filepath: str, waypoints) -> None:
    """
    Write (t_ns, JointState) pairs to a new bag at filepath.

    Parameters
    ----------
    filepath  : destination bag path; must not already exist
    waypoints : iterable of (t_ns: int, sensor_msgs/JointState) pairs, as
                buffered by MotionRecorder
    """
    with classic_bags.Bag(filepath, 'w') as bag:
        for t_ns, msg in waypoints:
            bag.write(TOPIC, msg, t_ns)


def _timestamp_to_ns(t) -> int:
    """
    Normalise a classic_bags read_messages() timestamp to integer
    nanoseconds.
    """
    if isinstance(t, int):
        return t
    if isinstance(t, float):
        return int(t * 1e9)
    if hasattr(t, 'nanoseconds'):  # rclpy.time.Time
        return int(t.nanoseconds)
    if hasattr(t, 'sec') and hasattr(t, 'nanosec'):  # builtin_interfaces/Time
        return int(t.sec) * 1_000_000_000 + int(t.nanosec)
    raise TypeError(f'Unrecognised classic_bags timestamp type: {type(t)!r}')


def load_waypoints(filepath: str):
    """
    Read /joint_states waypoints from a bag written by save_waypoints().

    Parameters
    ----------
    filepath : path to the bag produced by MotionRecorder

    Returns
    -------
    list of (t_elapsed_seconds, JointState msg) tuples, ordered as recorded,
    with t_elapsed measured from the first message's timestamp.
    """
    waypoints = []
    t0_ns = None
    with classic_bags.Bag(filepath, 'r') as bag:
        for _, msg, t in bag.read_messages(topics=[TOPIC]):
            t_ns = _timestamp_to_ns(t)
            if t0_ns is None:
                t0_ns = t_ns
            waypoints.append(((t_ns - t0_ns) / 1e9, msg))
    return waypoints
