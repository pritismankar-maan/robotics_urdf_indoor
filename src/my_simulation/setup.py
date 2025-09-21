from setuptools import setup
import os
from glob import glob 

package_name = 'my_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    
    # store all file names
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install world files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    	 # Install config files (controllers YAML)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    zip_safe=True,
    maintainer='pritismankar',
    maintainer_email='pritismankar@todo.todo',
    description='Gazebo simulation world and launch files',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'velocity_driver = my_simulation.velocity_driver:main',
      ],
    },
)
