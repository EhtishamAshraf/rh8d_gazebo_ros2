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
import xacro


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gz_args = LaunchConfiguration('gz_args', default='')

    # Controller and XACRO file paths
    control_pkg_path = get_package_share_directory('rh8d_control')
    robot_controllers = os.path.join(control_pkg_path, 'config', 'rh8dL_controllers_pos.yaml')

    desc_pkg_path = get_package_share_directory('rh8d_description')
    xacro_path = os.path.join(desc_pkg_path, 'urdf', 'rh8dL_main_pos.xacro')
    
    doc = xacro.process_file(
        xacro_path,
        mappings={'robot_controllers': robot_controllers}
    )
    robot_description_content = doc.toxml()

    # Passing the URDF content directly as the 'robot_description' parameter
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    # Robot State Publisher Node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],

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
        parameters=[{'use_sim_time': use_sim_time}],
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
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Bridge node for ROS-Gazebo communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
        PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'),
            'launch',
            'gz_sim.launch.py'
        ])
    ),
    launch_arguments={
        'gz_args': [gz_args, ' -r -v 1 /home/ehtisham/Desktop/Robotics_uclv/03_PROJECTS/P1_rh8d_sim/1-Gazebo/v2_rh8d_ws/src/rh8d_description/worlds/table_rubik.sdf']
    }.items(),

    )

    return LaunchDescription([
        
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gz_args', default_value=''),

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
    ])