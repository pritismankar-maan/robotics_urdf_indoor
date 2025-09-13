from setuptools import setup
import os
from glob import glob

package_name = 'my_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pritismankar',
    maintainer_email='pritismankar@todo.todo',
    description='TODO: SLAM',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
           'map_builder = my_slam.scripts.map_builder:main',
           'scan_matcher = my_slam.scripts.scan_matcher:main',	    	
        ],
    },
)
