import os
import platform
import shutil
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('diff_robot_sim')
    default_world = os.path.join(pkg_share, 'worlds', 'main.world')

    world = LaunchConfiguration('world')
    use_gui = LaunchConfiguration('use_gui')
    use_sim_time = LaunchConfiguration('use_sim_time')

    bridge_args = [
        '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        '/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
    ]
    bridge_remaps = [('/odometry', '/odom')]

    bridge_no_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args,
        remappings=bridge_remaps,
        output='screen',
        condition=UnlessCondition(use_sim_time),
    )

    bridge_with_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args + ['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        remappings=bridge_remaps,
        output='screen',
        condition=IfCondition(use_sim_time),
    )

    tf_params = [{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}]

    static_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.12', '0', '0', '0', 'base_footprint', 'base_link'],
        parameters=tf_params,
        output='screen',
    )

    tf_cam = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.36', '0', '0', '0', 'base_link', 'camera_link'],
        parameters=tf_params,
        output='screen',
    )

    tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.18', '0', '0.22', '0', '0', '0', 'base_link', 'lidar_link'],
        parameters=tf_params,
        output='screen',
    )

    tf_lidar_sensor = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'diff_robot/lidar_link/lidar'],
        parameters=tf_params,
        output='screen',
    )

    tf_lidar_sensor_cc = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'diff_robot/lidar_link/lidarcc'],
        parameters=tf_params,
        output='screen',
    )

    tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.08', '0', '0', '0', 'base_link', 'imu_link'],
        parameters=tf_params,
        output='screen',
    )

    odom_tf = Node(
        package='diff_robot_sim',
        executable='odom_tf_broadcaster.py',
        parameters=tf_params + [{
            'odom_topic': '/odom',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
        }],
        output='screen',
    )

    # Prefer a real gz executable (first in PATH), otherwise try CONDA_PREFIX.
    gz_bin = shutil.which('gz')
    if not gz_bin:
        # Try the current Python prefix (common for mamba envs)
        prefix_candidate = os.path.join(sys.prefix, 'bin', 'gz')
        if os.path.exists(prefix_candidate):
            gz_bin = prefix_candidate
        else:
            conda_prefix = os.environ.get('CONDA_PREFIX', '')
            gz_candidate = os.path.join(conda_prefix, 'bin', 'gz') if conda_prefix else ''
            gz_bin = gz_candidate if os.path.exists(gz_candidate) else 'gz'

    is_darwin = platform.system() == 'Darwin'

    if is_darwin:
        # macOS: run server and GUI as separate processes.
        gz_server = ExecuteProcess(
            cmd=[gz_bin, 'sim', '-r', '-s', '-v', '3', world],
            output='screen',
        )
        gz_gui = TimerAction(
            period='2.0',
            actions=[
                ExecuteProcess(
                    cmd=[gz_bin, 'sim', '-g', '-v', '3'],
                    output='screen',
                    condition=IfCondition(use_gui),
                )
            ],
        )
    else:
        # Other platforms can run GUI+server in a single process.
        gz_single = ExecuteProcess(
            cmd=[gz_bin, 'sim', '-r', '-v', '3', world],
            output='screen',
            condition=IfCondition(use_gui),
        )
        gz_server = ExecuteProcess(
            cmd=[gz_bin, 'sim', '-r', '-s', '-v', '3', world],
            output='screen',
            condition=UnlessCondition(use_gui),
        )

    gz_resource_path = os.pathsep.join([
        os.path.join(pkg_share, 'models'),
        os.path.join(pkg_share, 'worlds'),
        os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    ])

    env_vars = [
        # macOS GUI apps (Gazebo, RViz) can show in Dock without a window otherwise.
        SetEnvironmentVariable('QT_MAC_WANTS_LAYER', '1'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', gz_resource_path),
        SetEnvironmentVariable('GZ_TRANSPORT_DEFAULT_PORT', '11445'),
    ]

    launch_actions = [
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='SDF world to load in Gazebo'
        ),
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Attach Gazebo GUI client to the running server'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time (requires /clock bridge support)'
        ),
        *env_vars,
        gz_server,
        bridge_no_clock,
        bridge_with_clock,
        static_base,
        tf_cam,
        tf_lidar,
        tf_lidar_sensor,
        tf_lidar_sensor_cc,
        tf_imu,
        odom_tf,
    ]
    if is_darwin:
        launch_actions.append(gz_gui)
    else:
        launch_actions.insert(launch_actions.index(gz_server), gz_single)

    return LaunchDescription(launch_actions)
