from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# TODO: add the parameter module for the tactical_node and bluecar_tf
def generate_launch_description():
    # Get the paths to the individual launch files
    imu_launch_path = get_package_share_directory('outdoor_test') + '/launch/imu_launch.py'
    ekf_launch_path = get_package_share_directory('outdoor_test') + '/launch/ekf_launch.py'
    
    # Include the IMU launch file
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(imu_launch_path)
    )

    
    # Include the IMU filter node
    imu_filter = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        output='screen'
    )

    # Include the wheel odometry node
    wheel_odom = Node(
        package='wheel_odometry',
        executable='wheel_odometry_node',
        output='screen',
        parameters=[{'is_radio': 0}]
    )

    # Include the EKF robot_localization launch file
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ekf_launch_path)
    )

    # Include the tactical_node
    tactical_node = Node(
        package='tactical_node',
        executable='tactical_node',
        output='screen'
    )

    # Include the aeb_rover
    aeb_rover = Node(
        package='aeb_rover',
        executable='aeb_outdoor',
        output='screen'
    )

    # Include the control_rover
    ctrl_rover = Node(
        package='control_rover',
        executable='low_level_control',
        output='screen'
    )

    # Include the map_odom_tf
    map_odom_tf = Node(
        package='map_odom_tf',
        executable='odom_transformer',
        output='screen'
    )

    # Include the map to odom transformation node
    bluecar_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_map_to_baselink',
            arguments=['5', '0', '0', '3.14159', '0', '0', 'map', 'odom'],  # TODO: check the RPY frame, now 3.14 is set to the first entry to get the correct value
            output='screen'
    )

    # Include the map to external camera transformation node
    camera_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_map_to_camera',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera'],
            output='screen'
    )

    # Add a log message to indicate successful launch
    launch_complete_message = LogInfo(msg="All nodes and launch files have been successfully started!")

    # Combine everything in a single LaunchDescription
    return LaunchDescription([
        imu_launch,
        # imu_filter,
        wheel_odom,
        ekf_launch,
        aeb_rover,
        ctrl_rover,
        bluecar_tf,
        camera_tf,
        tactical_node,
        map_odom_tf,
        launch_complete_message
    ])
