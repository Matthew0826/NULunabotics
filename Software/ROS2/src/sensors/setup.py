from setuptools import find_packages, setup

package_name = 'sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'lunabotics_interfaces'],
    zip_safe=True,
    maintainer='nulunabotics-software-team',
    maintainer_email='selene@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar = sensors.lidar:main',
            'positioning = sensors.positioning:main',
            'power_data = sensors.power_data:main',
            'serial_port_server = sensors.serial_port_server:main',
            'mock_positioning = sensors.mock_positioning:main'
        ],
    },
)
