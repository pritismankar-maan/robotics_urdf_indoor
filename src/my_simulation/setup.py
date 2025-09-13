from setuptools import setup
import os
from glob import glob  # Required for data_files with directories

package_name = 'my_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install world files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    zip_safe=True,
    maintainer='pritismankar',
    maintainer_email='pritismankar@todo.todo',
    description='Gazebo simulation world and launch files',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],  # No scripts in this package yet
    },
)
