from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    config_file = os.path.join(
        get_package_share_directory('wheel_odometry'),
        'config',
        'wheel_odometry_config.yaml'
    )


    return LaunchDescription([
        Node(
            package='wheel_odometry',           # Your package name
            executable='wheel_odometry_node',   # The node executable name
            name='wheel_odometry_node',         # Node name
            output='screen',                    # Output logs to the screen
            parameters=[config_file]            # Load parameters from YAML
        )
    ])
