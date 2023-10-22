# Copyright 2020 F1TENTH Foundation
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#
#   * Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#
#   * Neither the name of the {copyright_holder} nor the names of its
#     contributors may be used to endorse or promote products derived from
#     this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    vesc_config_left = os.path.join(
        get_package_share_directory('vesc_driver'),
        'params',
        'vesc_config_left.yaml'
        )
    vesc_config_right = os.path.join(
        get_package_share_directory('vesc_driver'),
        'params',
        'vesc_config_right.yaml'
        )
    diff_drive_config = os.path.join(
        get_package_share_directory('diff_drive'),
        'config',
        'diff_drive_config.yaml'
        )
    return LaunchDescription([
        DeclareLaunchArgument(
            name="config_left",
            default_value=vesc_config_left,
            description="VESC yaml configuration file.",
        ),
        Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='vesc_driver_node_left',
            parameters=[LaunchConfiguration("config_left")]
        ),
        DeclareLaunchArgument(
            name="config_right",
            default_value=vesc_config_right,
            description="VESC yaml configuration file.",
        ),
        Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='vesc_driver_node_right',
            parameters=[LaunchConfiguration("config_right")]
        ),
        DeclareLaunchArgument(
            name="config",
            default_value=diff_drive_config,
            description="Differential Drive yaml configuration file.",
        ),
        Node(
            package='diff_drive',
            executable='diff_drive',
            name='diff_drive_node',
            parameters=[LaunchConfiguration("config")]
        ),
        Node(
            package='ldlidar_stl_ros2',
            executable='ldlidar_stl_ros2_node',
            name='LD19',
            output='screen',
            parameters=[
              {'product_name': 'LDLiDAR_LD19'},
              {'topic_name': 'scan'},
              {'frame_id': 'base_laser'},
              {'port_name': '/dev/ttyUSB0'},
              {'port_baudrate': 230400},
              {'laser_scan_dir': True},
              {'enable_angle_crop_func': False},
              {'angle_crop_min': 135.0},
              {'angle_crop_max': 225.0}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_laser_ld19',
            arguments=['0','0','0.18','0','0','0','base_link','base_laser']
        )
        Node(
            package='odometry_estimator',
            executable='odometry_estimator',
            name='odometry_estimator_node'
        )

    ])
