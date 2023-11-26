#!/bin/bash

echo "Sourcing ROS2 installation"
source install/setup.bash
echo "ROS2 sourced"

echo "Setting permissions for peripheral devices"
sudo chmod 777 /dev/ttyACM0	#(permissions for left/right vesc)
sudo chmod 777 /dev/ttyACM1	#(permissions for left/right vesc)
sudo chmod 777 /dev/ttyUSB0	#(permissions for lidar)
sudo chmod 777 /dev/ttyS0	#(permissions for IMU)
echo "Permissions set"

echo "Starting ROS2 node"
ros2 launch vesc_driver vesc_driver_node.launch.py