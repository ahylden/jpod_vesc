import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro

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
    odom_to_tf_config = os.path.join(
        get_package_share_directory('odom_to_tf_ros2'),
        'config',
        'odom_to_tf.yaml'
        )
    robot_descript = os.path.join(
        get_package_share_directory('vesc_driver'),
        'description',
        'robot.urdf.xacro'
        )
    robot_local_config = os.path.join(
        get_package_share_directory('vesc_driver'),
        'params',
        'ekf.yaml'
        )
    
    nav2_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch/navigation_launch.py')
            )
        )
    
    slam_toolbox_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch/online_async_launch.py')
            )
        )
    
    robot_description_config = xacro.process_file(robot_descript)
    params = {'robot_description': robot_description_config.toxml()}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
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
              {'enable_angle_crop_func': True},
              {'angle_crop_min': -90.0},
              {'angle_crop_max': 90.0}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_laser_ld19',
            arguments=['0.314','0','0.127','3.15','0','0','base_link','base_laser'] #yaw 3.15
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_footprint',
            arguments=['0','0','0','0','0','0','base_link','base_footprint']
        ),
        Node(
            package='odometry_estimator',
            executable='odometry_estimator',
            name='odometry_estimator_node'
        ),
        DeclareLaunchArgument(
            name="config_odom_tf",
            default_value=odom_to_tf_config,
            description="Odom transform yaml configuration file.",
        ),
        Node(
            package='odom_to_tf_ros2',
            executable='odom_to_tf',
            name='odom_to_tf_node',
            parameters=[LaunchConfiguration("config_odom_tf")]
        ),
        #Node(
        #    package='mpu_6050_driver',
        #    executable='imu_node',
        #    name='imu'
        #),
        #DeclareLaunchArgument(
        #    name="config_local",
        #    default_value=robot_local_config,
        #    description="Robot localization yaml configuration file.",
        #),
        #Node(
        #    package='robot_localization',
        #    executable='ekf_node',
        #    name='ekf_filter_node',
        #    output='screen',
        #    parameters=[LaunchConfiguration("config_local")]
        #),
        node_robot_state_publisher,
        nav2_launch_file,
        slam_toolbox_file
    ])
