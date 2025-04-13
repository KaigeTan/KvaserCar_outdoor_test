from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rviz_config_path = os.path.join(
        get_package_share_directory('visualize_car'),
        'rviz',
        'visualize_all.rviz'
    )

    return LaunchDescription([
        Node(
            package='visualize_car',
            executable='draw_markers_node',
            name='marker_node',
            output='screen'
        ),
        Node(
            package='visualize_car',
            executable='draw_cars_node',
            name='car_node',
            output='screen'
        ),
        Node(
            package='visualize_car',
            executable='fake_odom_node',
            name='fake_odom_node',
            output='screen'
        ),
         Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_map_to_base',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        ),
    ])
