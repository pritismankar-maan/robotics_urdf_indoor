import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Include your simulation launch if needed (optional)
    pkg_sim = get_package_share_directory('my_odometry')
    odom_launch = os.path.join(pkg_sim, 'launch', 'odom.launch.py')

    return LaunchDescription([
        # Optional: bring up Gazebo + robot + controllers
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(odom_launch)
        ),
        
        # Start your EKF fusion node
        Node(
            package='my_fusion',
            executable='ekf_fusion_node',
            name='ekf_fusion',
            output='screen',
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', 'info']
        ),
    ])
