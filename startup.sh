#!/bin/bash

source install/setup.bash

sudo chmod 777 /dev/ttyACM0	#(permissions for left/right vesc)
sudo chmod 777 /dev/ttyACM1	#(permissions for left/right vesc)
sudo chmod 777 /dev/ttyUSB0	#(permissions for lidar)
sudo chmod 777 /dev/ttyS0	#(permissions for IMU)

ros2 launch vesc_driver vesc_driver_node.launch.py