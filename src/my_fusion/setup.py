from setuptools import setup
import os
from glob import glob

package_name = 'my_fusion'

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
        # Install scripts (Python nodes)
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    zip_safe=True,
    maintainer='pritismankar',
    maintainer_email='pritismankar@todo.todo',
    description='Sensor fusion nodes (EKF)',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ekf_fusion = my_fusion.scripts.ekf_fusion:main',
        ],
    },
)
