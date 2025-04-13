from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'visualize_car'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    include_package_data=True,
    maintainer='tk22',
    maintainer_email='tk22@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'draw_markers_node = visualize_car.draw_markers_node:main',
            'draw_cars_node = visualize_car.draw_cars_node:main',
            'fake_odom_node = visualize_car.fake_odom_pub:main',
        ],
    },
)
