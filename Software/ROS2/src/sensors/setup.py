from setuptools import find_packages, setup

package_name = 'sensors'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['resource/serial_port_id_map.json']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'lunabotics_interfaces'],
    zip_safe=True,
    maintainer='nulunabotics-software-team',
    maintainer_email='spool.i@northeastern.edu',
    description='Processes data coming from the NU Lunabotics sensor boards over the serial ports.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_forwarder = sensors.lidar_forwarder:main',
            'positioning = sensors.positioning:main',
            'power_data = sensors.power_data:main',
            'mock_positioning = sensors.mock_positioning:main',
            'serial_port_manager = sensors.serial_port_manager:main',
            'battery = sensors.battery:main',
        ],
    },
)
