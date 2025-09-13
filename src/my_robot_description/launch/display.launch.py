import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_description')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'robot.urdf.xacro')
    robot_description = {'robot_description': Command(['xacro ', urdf_file])}
    rviz_config = os.path.join(pkg_dir, 'rviz', 'config.rviz')

    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen', parameters=[robot_description]),
        Node(package='joint_state_publisher', executable='joint_state_publisher', output='screen'),
        Node(package='rviz2', executable='rviz2', output='screen', arguments=['-d', rviz_config]),
    ])
