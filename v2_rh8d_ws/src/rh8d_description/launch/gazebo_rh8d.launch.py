from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_path = get_package_share_directory('rh8d_description')
    urdf_path = os.path.join(pkg_path, 'urdf', 'rh8dL_main.urdf')

    return LaunchDescription([

        # Start Gazebo (empty world)
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-v 4', '-r', 'empty.sdf'],
            output='screen'
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': open(urdf_path).read()
            }],
            output='screen'
        ),

        # Spawn robot into Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-world', 'empty',
                '-name', 'rh8d',
                '-topic', 'robot_description'
            ],
            output='screen'
        ),
    ])
