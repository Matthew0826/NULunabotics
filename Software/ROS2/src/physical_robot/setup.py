from setuptools import find_packages, setup

package_name = 'physical_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='selene',
    maintainer_email='selene@todo.todo',
    description='contains the operations of the robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_driver = physical_robot.motor_driver:main',
            'controller = physical_robot.controller:main',
        ],
    },
)
