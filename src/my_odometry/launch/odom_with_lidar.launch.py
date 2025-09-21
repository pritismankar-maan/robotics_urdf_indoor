import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Gazebo + robot launch
    pkg_sim = get_package_share_directory('my_simulation')
    sim_launch = os.path.join(pkg_sim, 'launch', 'sim_open_control.launch.py')
    #sim_launch = os.path.join(pkg_sim, 'launch', 'sim.launch.py')

    # Bag file directory
    pkg_odom = get_package_share_directory('my_odometry')
    bags_dir = os.path.join(pkg_odom, "bags")
    os.makedirs(bags_dir, exist_ok=True)
    bag_name = f"session_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    bag_path = os.path.join(bags_dir, bag_name)

    return LaunchDescription([
        # Launch simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch)
        ),

        # Odometry node
        Node(
            package='my_odometry',
            executable='odom_node',
            name='odom_node',
            output='screen',
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', 'debug']
        ),

        # LiDAR odometry node
        Node(
            package='my_odometry',
            executable='lidar_odometry_node',
            name='lidar_odometry_node',
            output='screen',
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', 'debug']
        ),

        # ROS2 bag recording
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '-o', bag_path,
                '/odom_est',
                '/odom_lidar_fused',
                '/tf',
            ],
            output='screen'
        ),
    ])

