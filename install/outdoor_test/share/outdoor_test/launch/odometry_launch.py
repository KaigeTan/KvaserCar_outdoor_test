from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

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
        output='screen'
    )

    # Include the EKF robot_localization launch file
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ekf_launch_path)
    )

    # Add a log message to indicate successful launch
    launch_complete_message = LogInfo(msg="All nodes and launch files have been successfully started!")

    # Combine everything in a single LaunchDescription
    return LaunchDescription([
        imu_launch,
        imu_filter,
        wheel_odom,
        ekf_launch,
        launch_complete_message
    ])
