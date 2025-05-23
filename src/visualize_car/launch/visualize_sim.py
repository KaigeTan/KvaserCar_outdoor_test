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
    
    # draw the car markers
    draw_car_node = Node(
        package='visualize_car',
        executable='draw_cars_node',
        name='car_node',
        output='screen'
        )
    
    # simulate the car trajectories
    sim_node = Node(
        package='visualize_car',
        executable='sim_node',
        name='sim_node',
        output='screen'
        )
    
    # setup tf tranform from map to baselink
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        output='screen'
        )
    
    # open rviz2 and launch the default setting
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
        )
    
    # Include the OBPS receiver node
    obps_receiver_tf = Node(
            package='obps_receiver',
            executable='obps_receiver',
            output='screen'
    )

    # Include the tactical_node
    tactical_node = Node(
        package='tactical_node',
        executable='tactical_node_new',
        output='screen',
        parameters=[{'ego_path_start': [5.0, 0.0]}] # TODO: placeholder, now I use tactical_node_new, since tactical_node has no parameter definition.
    )

    return LaunchDescription([obps_receiver_tf,
                              draw_car_node,
                              sim_node,
                              static_transform_publisher,
                              rviz_node,
                              tactical_node,
                              ])