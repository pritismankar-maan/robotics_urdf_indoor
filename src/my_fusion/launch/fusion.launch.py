import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Include your simulation launch if needed (optional)
    pkg_sim = get_package_share_directory('my_simulation')
    sim_launch = os.path.join(pkg_sim, 'launch', 'sim.launch.py')

    return LaunchDescription([
        # Optional: bring up Gazebo + robot + controllers
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch)
        ),

        # Start your odometry node (required for /odom_est)
        Node(
            package='my_odometry',
            executable='odom_node',
            name='odom_node',
            output='screen',
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', 'info']
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
