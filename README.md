# Kvaser car outdoor test ROS2 code
## How to run the ROS2 function
1. Power the rover with the battery and ensure the Ethernet cable is connected to the 5G router. Connect the computer to the wireless network:

* SSID: nvidia-hotspot
* Password: nvidia123

2. The default IP address of the rover is ```192.168.2.102```. Connect to the rover via SSH:
```bash
ssh nvidia@192.168.2.102
```
The password is ```nvidia```.

3. The workspace we use for this project is ```~/Kvaser_outdoor_test```. Navigate to the folder. If there is nothing changed in the code, source it with:
```bash
source ~/Kvaser_outdoor_test/install/setup.bash
```
And then launch the code with:
```bash
ros2 launch outdoor_test outdoor_launch
```


## aeb_rover
The AEB function used is ```aeb_outdoor.py```. It checks the ultra sonic sensor distance on the front and rear parts of the car.

The default safety distance threshold is 200 mm.
