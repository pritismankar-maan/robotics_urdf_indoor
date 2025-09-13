import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_urdf = get_package_share_directory('my_robot_description')
    pkg_gazebo = get_package_share_directory('gazebo_ros')
    world_file = os.path.join(get_package_share_directory('my_simulation'), 'worlds', 'apartment.world')
    urdf_file = os.path.join(pkg_urdf, 'urdf', 'robot.urdf.xacro')
    robot_description = {'robot_description': Command(['xacro ', urdf_file])}

    return LaunchDescription([
        # Gazebo
        IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(pkg_gazebo, 'launch'), '/gazebo.launch.py']),
                                 launch_arguments={'world': world_file}.items()),
        # Spawn Robot
        Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-entity', 'my_diffbot', '-topic', 'robot_description'],
             output='screen', parameters=[robot_description]),
        # Pub TF/State
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen', parameters=[robot_description]),
        Node(package='joint_state_publisher', executable='joint_state_publisher', output='screen'),
        # RViz
        Node(package='rviz2', executable='rviz2', output='screen', arguments=['-d', os.path.join(pkg_urdf, 'rviz', 'config.rviz')]),
    ])
