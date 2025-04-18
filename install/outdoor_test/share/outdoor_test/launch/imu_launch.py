# this is equivlent to the norlab_xsens_driver/launch/xsens_driver.launch.xml
# I modify it in ros2 launch format and change the necessary parameter values

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments (parameters)
    declare_device_arg = DeclareLaunchArgument('device', default_value='/dev/ttyUSB0', description='Device port')
    declare_baudrate_arg = DeclareLaunchArgument('baudrate', default_value='115200', description='Baud rate')
    declare_timeout_arg = DeclareLaunchArgument('timeout', default_value='0.002', description='Timeout')
    declare_initial_wait_arg = DeclareLaunchArgument('initial_wait', default_value='0.1', description='Initial wait time')
    declare_frame_id_arg = DeclareLaunchArgument('frame_id', default_value='/imu', description='Frame ID')
    declare_frame_local_arg = DeclareLaunchArgument('frame_local', default_value='ENU', description='Frame local')
    declare_no_rotation_duration_arg = DeclareLaunchArgument('no_rotation_duration', default_value='0', description='No rotation duration')
    declare_angular_velocity_cov_diag_arg = DeclareLaunchArgument(
        'angular_velocity_covariance_diagonal',
        default_value='[0.0004, 0.0004, 0.0004]',
        description='Angular velocity covariance diagonal'
    )
    declare_linear_acceleration_cov_diag_arg = DeclareLaunchArgument(
        'linear_acceleration_covariance_diagonal',
        default_value='[0.0004, 0.0004, 0.0004]',
        description='Linear acceleration covariance diagonal'
    )
    declare_orientation_cov_diag_arg = DeclareLaunchArgument(
        'orientation_covariance_diagonal',
        default_value='[0.01745, 0.01745, 0.15708]',
        description='Orientation covariance diagonal'
    )

    # Flush the serial port before running
    flush_serial = ExecuteProcess(
        cmd=['stty', '-F', LaunchConfiguration('device'), 'clocal', 'crtscts'],
        output='screen',
        log_cmd=True,  # Log the command execution for debugging
    )

    # Add a delay to ensure flush completes before starting the IMU node
    start_imu_node_after_flush = TimerAction(
        period=2.0,  # Wait for 2 seconds after flushing
        actions=[
            Node(
                package='xsens_driver',
                executable='mtnode.py',
                name='xsens_driver',
                output='screen',
                parameters=[{
                    'device': LaunchConfiguration('device'),
                    'baudrate': LaunchConfiguration('baudrate'),
                    'timeout': LaunchConfiguration('timeout'),
                    'initial_wait': LaunchConfiguration('initial_wait'),
                    'frame_id': LaunchConfiguration('frame_id'),
                    'frame_local': LaunchConfiguration('frame_local'),
                    'no_rotation_duration': LaunchConfiguration('no_rotation_duration'),
                    'angular_velocity_covariance_diagonal': LaunchConfiguration('angular_velocity_covariance_diagonal'),
                    'linear_acceleration_covariance_diagonal': LaunchConfiguration('linear_acceleration_covariance_diagonal'),
                    'orientation_covariance_diagonal': LaunchConfiguration('orientation_covariance_diagonal'),
                }],
                remappings=[
                    ('/imu/data', '/imu/data_raw')  # Remap topics, as the imu filter node scuscribes /imu/data_raw
                ],
                on_exit=Shutdown()  # Stop the launch if this node fails
            )
        ]
    )

    # Add a static transform publisher to define the transformation from base_link to imu
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_base_to_imu',
        arguments=['-0.2', '0.0', '0.22', '0', '0', '3.1416', 'base_link', 'imu'], # this is a fixed tf from base_link (car chassis) to imu, note imu mount yaw is ratated
        output='screen',
        on_exit=Shutdown()  # Stop the launch if this node fails
    )

    # Combine everything into the launch description
    return LaunchDescription([
        declare_device_arg,
        declare_baudrate_arg,
        declare_timeout_arg,
        declare_initial_wait_arg,
        declare_frame_id_arg,
        declare_frame_local_arg,
        declare_no_rotation_duration_arg,
        declare_angular_velocity_cov_diag_arg,
        declare_linear_acceleration_cov_diag_arg,
        declare_orientation_cov_diag_arg,
        flush_serial,
        start_imu_node_after_flush,
        static_transform_publisher
    ])
