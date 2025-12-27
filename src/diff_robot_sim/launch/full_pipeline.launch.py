import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('diff_robot_sim')

    world_default = os.path.join(pkg_share, 'worlds', 'main.world')
    nav2_params_default = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    slam_params_default = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')
    rviz_config_default = os.path.join(pkg_share, 'config', 'slam.rviz')

    world = LaunchConfiguration('world')
    use_gui = LaunchConfiguration('use_gui')
    use_slam = LaunchConfiguration('use_slam')
    use_rviz = LaunchConfiguration('use_rviz')
    use_nav2 = LaunchConfiguration('use_nav2')
    enable_teleop = LaunchConfiguration('enable_teleop')
    enable_autonomy = LaunchConfiguration('enable_autonomy')
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_delay = LaunchConfiguration('nav2_delay')
    rviz_delay = LaunchConfiguration('rviz_delay')
    autonomy_delay = LaunchConfiguration('autonomy_delay')
    nav2_params = LaunchConfiguration('nav2_params_file')
    slam_params = LaunchConfiguration('slam_params_file')
    map_yaml = LaunchConfiguration('map')
    rviz_config = LaunchConfiguration('rviz_config')

    # Usage:
    #   ros2 launch diff_robot_sim full_pipeline.launch.py use_gui:=true use_rviz:=true
    # Verify:
    #   ros2 topic echo /scan
    #   ros2 topic list | rg map
    #   ros2 run tf2_ros tf2_echo odom base_link
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'sim_launch.py'])
        ),
        launch_arguments={
            'world': world,
            'use_gui': use_gui,
            'use_sim_time': use_sim_time,
        }.items()
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'nav2_launch.py'])
        ),
        launch_arguments={
            'use_slam': use_slam,
            'map': map_yaml,
            'params_file': nav2_params,
            'use_rviz': use_rviz,
            'use_sim_time': use_sim_time,
        }.items()
    )

    slam_only_condition = IfCondition(PythonExpression([
        "'", use_nav2, "'.lower() == 'false' and '",
        enable_autonomy, "'.lower() == 'false'"
    ]))

    nav2_or_autonomy = PythonExpression([
        "'", use_nav2, "'.lower() == 'true' or '",
        enable_autonomy, "'.lower() == 'true'"
    ])

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'slam_launch.py'])
        ),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': use_sim_time,
        }.items(),
        condition=slam_only_condition
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
        additional_env={'QT_MAC_WANTS_LAYER': '1'},
        output='screen',
        emulate_tty=True,
    )

    rviz = TimerAction(
        period=rviz_delay,
        actions=[
            rviz_node,
        ],
        condition=IfCondition(PythonExpression([
            "'", use_nav2, "'.lower() == 'false' and '",
            enable_autonomy, "'.lower() == 'false' and '",
            use_rviz, "'.lower() == 'true'"
        ])),
    )

    teleop = ExecuteProcess(
        cmd=[
            'ros2',
            'run',
            'diff_robot_sim',
            'wasd_teleop.py',
        ],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(PythonExpression([
            "'", enable_teleop, "'.lower() == 'true' and '",
            enable_autonomy, "'.lower() == 'false'"
        ])),
    )

    frontier_explorer = TimerAction(
        period=autonomy_delay,
        actions=[
            Node(
                package='diff_robot_sim',
                executable='frontier_explorer.py',
                output='screen',
                parameters=[
                    {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
                    {'map_topic': '/map'},
                ],
            )
        ],
        condition=IfCondition(enable_autonomy),
    )

    return LaunchDescription([
        SetEnvironmentVariable('QT_MAC_WANTS_LAYER', '1'),
        LogInfo(msg='SLAM-only demo: expect /scan, /odom, /map and TF odom->base_link, map->odom.'),
        LogInfo(msg='Teleop disabled by default. Enable with enable_teleop:=true (WASD in this terminal).'),
        DeclareLaunchArgument(
            'world',
            default_value=world_default,
            description='World file to load'
        ),
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Attach Gazebo GUI'
        ),
        DeclareLaunchArgument(
            'use_slam',
            default_value='true',
            description='Enable SLAM toolbox (otherwise expects static map)'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start Nav2 RViz (for map building + goals)'
        ),
        DeclareLaunchArgument(
            'use_nav2',
            default_value='false',
            description='Launch Nav2 stack (off by default for SLAM-only runs)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time (requires /clock bridge support)'
        ),
        DeclareLaunchArgument(
            'enable_teleop',
            default_value='false',
            description='Launch teleop_twist_keyboard for SLAM-only demo'
        ),
        DeclareLaunchArgument(
            'enable_autonomy',
            default_value='false',
            description='Enable frontier-based autonomous exploration (requires Nav2)'
        ),
        DeclareLaunchArgument(
            'nav2_delay',
            default_value='2.0',
            description='Delay (sec) before starting Nav2'
        ),
        DeclareLaunchArgument(
            'rviz_delay',
            default_value='2.5',
            description='Delay (sec) before starting RViz'
        ),
        DeclareLaunchArgument(
            'autonomy_delay',
            default_value='6.0',
            description='Delay (sec) before starting frontier explorer'
        ),
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Static map YAML when SLAM disabled'
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=nav2_params_default,
            description='Nav2 params YAML'
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=slam_params_default,
            description='SLAM Toolbox params YAML'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_config_default,
            description='RViz config for SLAM-only mode'
        ),
        sim_launch,
        slam_launch,
        rviz,
        teleop,
        TimerAction(
            period=nav2_delay,
            actions=[nav2_launch],
            condition=IfCondition(nav2_or_autonomy),
        ),
        frontier_explorer,
    ])
