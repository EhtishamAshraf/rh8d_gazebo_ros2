from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    gz_args = LaunchConfiguration('gz_args', default='')

    # Controller and URDF file paths
    control_pkg_path = get_package_share_directory('rh8d_control')
    robot_controllers = os.path.join(control_pkg_path, 'config', 'rh8dL_controllers.yaml')

    desc_pkg_path = get_package_share_directory('rh8d_description')
    urdf_path = os.path.join(desc_pkg_path, 'urdf', 'rh8dL_main.urdf')
    with open(urdf_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    # Passing the URDF content directly as the 'robot_description' parameter
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # Spawn robot into Gazebo
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'rh8dL',
            '-allow_renaming', 'true'
        ],
    )

    # Controllers
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster'],
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_trajectory_controller',
            '--param-file',
            robot_controllers
        ],
    )

    # Bridge node for ROS-Gazebo communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                   'launch',
                                   'gz_sim.launch.py'])]
        ),
        launch_arguments=[('gz_args', [gz_args, ' -r -v 1 empty.sdf'])],
    )

    return LaunchDescription([
        gz_sim,
        bridge,
        node_robot_state_publisher,
        gz_spawn_entity,

        RegisterEventHandler(
            OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[joint_trajectory_controller_spawner],
            )
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'
        ),
    ])
