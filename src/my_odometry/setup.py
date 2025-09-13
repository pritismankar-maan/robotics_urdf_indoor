from setuptools import setup
import os
from glob import glob  # For installing directories like scripts/launch

package_name = 'my_odometry'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Install scripts (makes them executable via entry_points)
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    # Remove install_requires—unneeded in ROS 2
    zip_safe=True,
    maintainer='pritismankar',
    maintainer_email='pritismankar@todo.todo',
    description='Odometry nodes for encoder and IMU',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'encoder_odom = my_odometry.scripts.encoder_odom:main',
            'imu_odom = my_odometry.scripts.imu_odom:main',
            'move_robot = my_odometry.scripts.move_robot:main',
        ],
    },
)
