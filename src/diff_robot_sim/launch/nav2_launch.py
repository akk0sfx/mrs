import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('diff_robot_sim')
    nav2_params_default = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    use_slam = LaunchConfiguration('use_slam')
    use_rviz = LaunchConfiguration('use_rviz')
    map_yaml = LaunchConfiguration('map')
    nav2_params = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # nav2_bringup (Jazzy) expects PythonExpression-friendly booleans ("True"/"False")
    slam_bool = PythonExpression([
        '"True" if "', use_slam, '".lower() in ["true","1","yes","on"] else "False"'
    ])

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam': slam_bool,
            'map': map_yaml,
            'params_file': nav2_params,
            'autostart': 'true'
        }.items()
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('nav2_bringup'),
                'launch',
                'rviz_launch.py'
            ])
        ),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_namespace': 'false',
            'namespace': ''
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_slam',
            default_value='true',
            description='Enable SLAM toolbox inside Nav2 bringup'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch Nav2 RViz view'
        ),
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Static map to use when SLAM is disabled'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=nav2_params_default,
            description='Nav2 parameter file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        bringup,
        rviz,
    ])
