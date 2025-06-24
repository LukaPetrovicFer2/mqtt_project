from setuptools import find_packages, setup

package_name = 'sensor_simulator'

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
    maintainer='luka',
    maintainer_email='luka.petrovic98@gmail.com',
    description='Simulated ROS2 sensor publisher for testing MQTT bridge',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_simulator = sensor_simulator.sensor_simulator:main',
        ],
    },
)
