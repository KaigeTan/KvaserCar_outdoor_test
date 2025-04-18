from setuptools import find_packages, setup

package_name = 'aeb_rover'

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
    maintainer='nvidia',
    maintainer_email='kaiget@kth.se',
    description='Autonomous Emergency Braking with Ultrasonic Sensors',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aeb_node_open = aeb_rover.aeb_rover_node:main',  # open loop joystick control with AEB 
            'aeb_node_closed = aeb_rover.PIctrl:main',  # closed loop control by PI with AEB
            'aeb_outdoor = aeb_rover.aeb_outdoor:main',  # outdoor test with AEB node
        ],
    },
)
