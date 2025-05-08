# Kvaser car outdoor test ROS2 code
## How to run the ROS2 function
1. Power the rover with the battery and ensure the Ethernet cable is connected to the 5G router. Connect the computer to the wireless network:

* SSID: nvidia-hotspot
* Password: nvidia123

2. The default IP address of the rover is `192.168.2.102`. Connect to the rover via SSH:
```bash
ssh nvidia@192.168.2.102
```
The password is `nvidia`.

3. The workspace we use for this project is `~/KvaserCar_outdoor_test`. Navigate to the folder. If there is nothing changed in the code, source it with:
```bash
source ~/KvaserCar_outdoor_test/install/setup.bash
```
And then launch the code with:
```bash
ros2 launch outdoor_test outdoor_launch.py
```


## aeb_rover
* The AEB function used for this project is [aeb_outdoor.py](./src/aeb_rover/aeb_rover/aeb_outdoor.py). It checks the ultrasonic sensor distance on the front and rear parts of the car. If the detected distance is smaller than the threshold, then a boolean True output is published to `/aeb_triggered` topic.

* The default safety distance threshold is 200 mm. It can be either changed in the `aeb_outdoor.py`: `self.declare_parameter('distance_threshold', 200.0)`

Or changed in the [launch file](./src/outdoor_test/launch/outdoor_launch.py):
```python
aeb_rover = Node(
        package='aeb_rover',
        executable='aeb_outdoor',
        output='screen',
        parameters=[{'distance_threshold': 100.0}]  # Override value here
    )
```

**TODO**: During the tests, we noticed there are some false positive detections of the ultrasonic sensor to trigger AEB. A signal filtering function is needed to improve this.

## control_rover
* The rover control node checks the reference speed given by `/ref_spd` topic and publishes the control signal for throttle in the topic `/rover/throttle`. If `/aeb_triggered` is True, the rover control node will stop the car by giving `/rover/throttle` with 0.

* `/rover/throttle` is a percentage value ranging from 0 to 100. Currently, we use a mapping function to convert the stable speed of the car to the throttle value. 

**TODO**: In the future, if accurate velocity tracking is needed, a PI controller can be added to the function by subscribing to the x-axis velocity of `/odometry/filtered`.

## tactical_node
developed by Gianfilippo.

## wheel_odometry
* The wheel odometry function estimates the rover's pose and velocity using rear wheel speed and steering angle inputs. It subscribes to `/rover/wheel_rear_left/speed_kph` and `/rover/wheel_rear_right/speed_kph`, estimates `x`, `y`, and `yaw (theta)` using a basic kinematic bicycle model, and publishes odometry to `/odometry` topic.

* Since the wheel encoder does not have the direction (`/rover/wheel_rear_left/speed_kph` and `/rover/wheel_rear_right/speed_kph` only give positive values). In the function, we use a state machine to map the `/rover/throttle` to the directions of the vehicle.

* In the launch file, we set `parameters=[{'is_radio': 0}]` since the vehicle is not controlled by radio but by **tactical_node**. If the rover is controlled by the radio, then the state machine will use `/rover/radio/throttle` but not `/rover/throttle`.

## outdoor_test
* The outdoor test is a launch package, where we use [outdoor_launch.py](./src/outdoor_test/launch/outdoor_launch.py) for experiments. Besides the function described above, we also use [XSENS IMU mti ROS2 driver](https://github.com/norlab-ulaval/norlab_xsens_driver), [imu_complementary_filter](https://github.com/CCNYRoboticsLab/imu_tools/tree/humble), and [EKF filter](https://github.com/cra-ros-pkg/robot_localization).

* The final estimated state of the rover is in `/odometry/filtered`, which is from EKF filter by combining the odometry estimation from IMU and wheel odom.

* We create two static coordination transformations in the launch file, and the current tf structure is [here](./frames_tf.pdf). The transformation definition may be changed in the future, depending on the experimental setup.

* Note that, in [EKF configuration file](./src/outdoor_test/config/ekf_config.yaml), we change the world frame to map `world_frame: map`. Thus, `/odometry/filtered` topic is in `map` coordinate but not `odom` coordinate.

* Note for XSENS IMU sdk, we changed one line of code in [mtnode.py](./src/norlab_xsens_driver/xsens_driver/mtnode.py):
```python
aself.frame_id = self.get_param('frame_id', 'base_imu').lstrip('/')
```
This is because EKF filter in robot localization needs `imu` as tf but not `/imu`. **mtnode.py** hardcoded it so we need to change manually.
