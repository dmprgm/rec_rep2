"""
Shared rosbag2 helpers for writing/reading the recorded /joint_states
trajectory. Bags are plain single-topic rosbag2 (sqlite3) bags -- they can
be inspected with `ros2 bag info <path>` like any other rosbag2 recording.
"""

from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import JointState

import rosbag2_py

TOPIC = '/joint_states'
MSG_TYPE = 'sensor_msgs/msg/JointState'
STORAGE_ID = 'sqlite3'


def _converter_options():
    return rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )


def save_waypoints(filepath: str, waypoints) -> None:
    """
    Write (t_ns, JointState) pairs to a new rosbag2 bag at filepath.

    Parameters
    ----------
    filepath  : destination bag directory; must not already exist
    waypoints : iterable of (t_ns: int, sensor_msgs/JointState) pairs, as
                buffered by MotionRecorder
    """
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=filepath, storage_id=STORAGE_ID),
        _converter_options(),
    )
    writer.create_topic(rosbag2_py.TopicMetadata(
        name=TOPIC,
        type=MSG_TYPE,
        serialization_format='cdr',
        offered_qos_profiles='',
    ))
    for t_ns, msg in waypoints:
        writer.write(TOPIC, serialize_message(msg), t_ns)
    # SequentialWriter has no explicit close(): the bag is flushed and
    # closed when the writer is destroyed, so drop the reference here
    # rather than leaving it open for the rest of the caller's scope.
    del writer


def load_waypoints(filepath: str):
    """
    Read /joint_states waypoints from a bag written by save_waypoints().

    Parameters
    ----------
    filepath : path to the bag directory produced by MotionRecorder

    Returns
    -------
    list of (t_elapsed_seconds, JointState msg) tuples, ordered as recorded,
    with t_elapsed measured from the first message's timestamp.
    """
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=filepath, storage_id=STORAGE_ID),
        _converter_options(),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[TOPIC]))

    waypoints = []
    t0_ns = None
    while reader.has_next():
        _, data, t_ns = reader.read_next()
        if t0_ns is None:
            t0_ns = t_ns
        msg = deserialize_message(data, JointState)
        waypoints.append(((t_ns - t0_ns) / 1e9, msg))
    return waypoints
