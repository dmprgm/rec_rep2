from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from rec_rep2.paths import BAGS_DIR

def generate_launch_description():

    # --- Declare arguments users can override on the CLI ---
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.10',
        description='IP address of the Kinova Gen3',
    )
    use_fake_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false',
        description='Use mock hardware for testing without robot',
    )
    save_directory_arg = DeclareLaunchArgument(
        'save_directory',
        default_value=BAGS_DIR,
        description='Directory recorded trajectory bags are saved to',
    )
    # See docs/cpp_servoing.md. Meaningless combined with use_fake_hardware
    # -- the C++ node has no fake-hardware simulation path by design; the
    # Python compliant_torque_mode.py's _fake_loop stays the only one.
    use_cpp_servo_backend_arg = DeclareLaunchArgument(
        'use_cpp_servo_backend',
        default_value='false',
        description=(
            'Drive compliant torque mode via the C++ compliant_torque_node '
            '(rec_rep2_servo) instead of the in-process Python control loop. '
            'Not meaningful together with use_fake_hardware.'
        ),
    )

    # --- Include the kortex_bringup launch file, launch file inception ---
    kortex_dir = get_package_share_directory('kortex_bringup')
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(kortex_dir, 'launch', 'gen3.launch.py')
        ),
        launch_arguments={
            'robot_ip':           LaunchConfiguration('robot_ip'),
            'use_fake_hardware':  LaunchConfiguration('use_fake_hardware'),
            'dof':                '7',
            'arm':                'gen3',
        }.items(),
    )

    # --- Your recorder node ---
    recorder_node = Node(
        package='rec_rep2',
        executable='recorder',
        name='motion_recorder',
        output='screen',
        parameters=[{
            'save_directory': LaunchConfiguration('save_directory'),
            'use_cpp_servo_backend': LaunchConfiguration('use_cpp_servo_backend'),
        }],
    )

    # --- Optional C++ low-level servoing backend (rec_rep2_servo) ---
    # See docs/cpp_servoing.md. Only started if use_cpp_servo_backend:=true;
    # robot_ip is shared with the driver/recorder above.
    compliant_torque_node = Node(
        package='rec_rep2_servo',
        executable='compliant_torque_node',
        name='compliant_torque_servo',
        output='screen',
        parameters=[{
            'robot_ip': LaunchConfiguration('robot_ip'),
        }],
        condition=IfCondition(LaunchConfiguration('use_cpp_servo_backend')),
    )

    return LaunchDescription([
        robot_ip_arg,
        use_fake_arg,
        save_directory_arg,
        use_cpp_servo_backend_arg,
        driver_launch,
        recorder_node,
        compliant_torque_node,
    ])