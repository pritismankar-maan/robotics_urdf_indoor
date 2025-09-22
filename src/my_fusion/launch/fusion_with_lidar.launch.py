import os
from datetime import datetime 
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Include your Odometry launch, Rviz saved file and EKF related Config file
    pkg_sim = get_package_share_directory('my_odometry')
    pkg_fusion = get_package_share_directory('my_fusion')
    odom_launch = os.path.join(pkg_sim, 'launch', 'odom_with_lidar_norecord.launch.py')
    fusion_config = os.path.join(pkg_fusion, 'config', 'ekf.yaml')
    rviz_config = os.path.join(pkg_fusion, 'rviz', 'fusion.rviz')
    
    # bag file related configs
    bags_dir = os.path.join(pkg_fusion, "bags")
    os.makedirs(bags_dir, exist_ok=True)
    bag_name = f"session_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    bag_path = os.path.join(bags_dir, bag_name)	
    
    return LaunchDescription([
        # Bring up Gazebo + robot + controllers + convert to odom from encoders
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
            parameters=[fusion_config],
            arguments=['--ros-args', '--log-level', 'info']
        ),
        
        # Start RViz with saved layout
        #Node(
        #    package='rviz2',
        #    executable='rviz2',
        #    name='rviz2_fusion',
        #    arguments=['-d', rviz_config],
        #    output='screen'
        #),
        
        # Start recording all odom topic and TFs in a bag file
        ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', bag_path,
            '/odom_est',
            '/odom_lidar_fused',
            '/odom_imu_est',
            '/odom_fused',
        ],
        output='screen'
        ),
    ])
