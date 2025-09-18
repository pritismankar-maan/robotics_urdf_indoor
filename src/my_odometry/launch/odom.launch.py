import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_sim = get_package_share_directory('my_simulation')
    sim_launch = os.path.join(pkg_sim, 'launch', 'sim.launch.py')

    return LaunchDescription([
        # Bring up Gazebo + robot + controllers
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch)
        ),

        # Start your odometry / trajectory node
        Node(
            package='my_odometry',
            executable='odom_node',
            name='trajectory_node',
            output='screen',
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', 'debug']
        ),
        
        Node(
            package='my_odometry',
            executable='live_plot',  # matches entry point
            name='live_plot_node',
        ),
    ])

