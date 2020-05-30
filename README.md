# ros2_gps_ws
GPS related code with simple ROS2 implementation. Includes NTRIP client and serial read/write to GPS device.

### Build
```
colcon build --symlink-install
source ~/workspace/ros2_gps_ws/install/setup.zsh
```
#### Open connection to caster and begin publishing data
```
ros2 launch ntrip_client ntrip_client.launch.py 
```
#### Open connection to RTK device and begin providing corrections / outputting NMEA buffer 
```
ros2 run serial_if serial_if
```
