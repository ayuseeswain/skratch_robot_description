import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    package_name = 'skratch_description'

    pkg_path = get_package_share_directory(package_name)
    xacro_file = os.path.join(pkg_path, 'description', 'skratch_robot.urdf.xacro')
    rviz_config_file = os.path.join(pkg_path, 'config', 'simulation.rviz')

    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'kinova_arm_controller'],
        output='screen'
    )

    load_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'kinova_gripper_controller'],
        output='screen'
    )
    
    static_tf_base_footprint_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        output='screen'
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': os.path.join(pkg_path, 'config', 'simple_environment.sdf')
        }.items()
    )

    # Spawn robot entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'skratch'],
        output='screen',
    )

    lidar_remap_node = Node(
        package='topic_tools',
        executable='relay',
        name='lidar_remap',
        parameters=[{'input_topic': '/lidar_front/out', 'output_topic': '/scan'}],
        output='screen'
    )
    
    delayed_spawn = TimerAction(
        period=1.0,
        actions=[spawn_entity]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        robot_state_publisher,
        gazebo_launch,
        delayed_spawn,

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_arm_controller]
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_arm_controller,
                on_exit=[load_gripper_controller]
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_gripper_controller,
                on_exit=[load_joint_state_controller]
            )
        ),

        static_tf_base_footprint_to_base_link,
        lidar_remap_node

    ])
