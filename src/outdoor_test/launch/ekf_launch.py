from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file = get_package_share_directory('outdoor_test') + '/config/ekf_config.yaml'

    return LaunchDescription([
        # EKF robot_localization Node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config_file],  # Pass the path to the YAML file
        ),
    ])
