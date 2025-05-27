from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml
from launch.actions import ExecuteProcess
import os
from datetime import datetime

# TODO: add the parameter module for the tactical_node and bluecar_tf
def generate_launch_description():
    # Get the paths to the individual launch files
    imu_launch_path = get_package_share_directory('outdoor_test') + '/launch/imu_launch.py'
    ekf_launch_path = get_package_share_directory('outdoor_test') + '/launch/ekf_launch.py'

    # Get the paths to the individual launch files
    params_ros_file = get_package_share_directory('outdoor_test') + '/config/params_ros.yaml'
    params_static_file = get_package_share_directory('outdoor_test') + '/config/params_static.yaml'
    imu_launch_path = get_package_share_directory('outdoor_test') + '/launch/imu_launch.py'
    ekf_launch_path = get_package_share_directory('outdoor_test') + '/launch/ekf_launch.py'
    
    # Load tranformation parameter from yaml file
    with open(params_static_file, 'r') as f:
        config = yaml.safe_load(f)
    tf_config_odom = config.get('static_transform_odom', {})
    # Convert values to strings for arguments
    tf_odom_args = [
        str(tf_config_odom.get('x', 0.0)), # change it to use the params_ros.yaml, construct myself
        str(tf_config_odom.get('y', 0.0)),
        str(tf_config_odom.get('z', 0.0)),
        str(tf_config_odom.get('roll', 0.0)),
        str(tf_config_odom.get('pitch', 0.0)),
        str(tf_config_odom.get('yaw', 0.0)),
        tf_config_odom.get('parent_frame', 'map'),
        tf_config_odom.get('child_frame', 'odom')
    ]
    
    tf_config_camera = config.get('static_transform_camera', {})
    # Convert values to strings for arguments
    tf_camera_args = [
        str(tf_config_camera.get('x', 0.0)),
        str(tf_config_camera.get('y', 0.0)),
        str(tf_config_camera.get('z', 0.0)),
        str(tf_config_camera.get('roll', 0.0)),
        str(tf_config_camera.get('pitch', 0.0)),
        str(tf_config_camera.get('yaw', 0.0)),
        tf_config_camera.get('parent_frame', 'map'),
        tf_config_camera.get('child_frame', 'odom')
    ]
    
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
        name='rover_odometry_node',
        output='screen',
        parameters=[params_ros_file]
    )

    # Include the EKF robot_localization launch file
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ekf_launch_path)
    )

    # Include the tactical_node
    tactical_node = Node(
        package='tactical_node',
        executable='tactical_node', # TODO: do we run tactical node or tactical node new?
        name='tactical_node',
        output='screen',
        parameters=[params_ros_file]
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
        executable='low_level_control_feedback', # 'low_level_control',
        output='screen',
        parameters=[params_ros_file]
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
            arguments=tf_odom_args,  # TODO: check the RPY frame, now 3.14 is set to the first entry to get the correct value
            output='screen'
    )

    # Include the map to external camera transformation node
    camera_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_map_to_camera',
            arguments=tf_camera_args,
            output='screen'
    )

    # Include the OBPS receiver node
    obps_receiver = Node(
            package='obps_receiver',
            executable='obps_receiver',
            name='obps_receiver',
            output='screen',
            parameters=[params_ros_file]
    )

    # Base folder to store rosbag files
    bag_base_dir = os.path.expanduser('~/KvaserCar_outdoor_test/recorded_rosbag')

    # Generate date-based subfolder (e.g., 0513)
    date_str = datetime.now().strftime('%m%d')
    bag_subdir = os.path.join(bag_base_dir, date_str)

    # Ensure directory exists
    os.makedirs(bag_subdir, exist_ok=True)

    # Full path for output rosbag (timestamped inside the date folder)
    bag_output_path = os.path.join(bag_subdir, 'rosbag_' + datetime.now().strftime('%H%M%S'))

    # Define the bag recording process
    rosbag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            'a',
            '-o', bag_output_path
        ],
        output='screen'
    )

    # Add a log message to indicate successful launch
    launch_complete_message = LogInfo(msg="All nodes and launch files have been successfully started!")

    # Combine everything in a single LaunchDescription
    return LaunchDescription([
        imu_launch,
        imu_filter,
        wheel_odom,
        ekf_launch,
        aeb_rover,
        ctrl_rover,
        obps_receiver,
        bluecar_tf,
        camera_tf,
        tactical_node,
        map_odom_tf,
        rosbag_record,
        launch_complete_message
    ])
