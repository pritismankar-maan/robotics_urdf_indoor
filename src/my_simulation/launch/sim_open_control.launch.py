import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command

def generate_launch_description():
    pkg_urdf = get_package_share_directory('my_robot_description')
    pkg_gazebo = get_package_share_directory('gazebo_ros')
    pkg_sim = get_package_share_directory('my_simulation')

    # prepare all file names
    world_file = os.path.join(pkg_sim, 'worlds', 'apartment.world')
    urdf_file = os.path.join(pkg_urdf, 'urdf', 'robot.urdf.xacro')
    controller_yaml = os.path.join(pkg_sim, 'config', 'diffbot_controllers.yaml')

    robot_description = {'robot_description': Command(['xacro ', urdf_file])}

    return LaunchDescription([

        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_file}.items()
        ),

        # Spawn Robot in Gazebo (slight delay to ensure Gazebo is ready)
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=['-entity', 'my_diffbot', '-topic', 'robot_description'],
                    output='screen',
                    parameters=[robot_description]
                )
            ]
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': True}]
        ),

        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),


        # Delay spawner to ensure ros2_control_node is fully up
        TimerAction(
            period=8.0,  # Foxy is slow to initialize
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner.py',
                    arguments=["diff_cont"],
                    output='screen',
                )
            ]
        ),
        
        # Delay spawner to ensure ros2_control_node is fully up
        TimerAction(
            period=8.0,  # Foxy is slow to initialize
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner.py',
                    arguments=["joint_broad"],
                    output='screen',
                )
            ]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_urdf, 'rviz', 'config.rviz')]
        ),
        
        # Start the velocity driver after controllers are ready
        TimerAction(
            period=10.0,  # wait until controllers and gazebo are up
            actions=[
                Node(
                    package='my_simulation',
                    executable='velocity_driver',
                    name='velocity_driver',
                    parameters=[{'use_sim_time': True}],
                    output='screen',
                )
            ]
        ),
    ])

