from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'lunabotics_interfaces'],
    zip_safe=True,
    maintainer='nulunabotics-software-team',
    maintainer_email='spool.i@northeastern.edu',
    description='Handles autonomous navigation of the robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry = navigation.odometry:main',
            'obstacle_detector = navigation.obstacle_detector:main',
            'pathfinder = navigation.pathfinder:main',
            'mock_odometry = navigation.mock_odometry:main',
            'planner = navigation.planner:main',
            'mock_obstacle_detector = navigation.mock_obstacle_detector:main',        
            ],
    },
)
