import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    slam_params_file = os.path.join(
        get_package_share_directory('skratch_description'),
        'config', 'slam_parameters.yaml'
    )

    nav2_launch_file = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch', 'navigation_launch.py'
    )

    slam_launch_file = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch', 'online_async_launch.py'
    )

    rviz_config_file = '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz'

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': slam_params_file
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_nav2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time if true'),
        slam_toolbox_launch,
        nav2_launch,
        rviz_node
    ])
