from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    # Nodes
    sabertooth_cmd_vel_bridge_node = Node(
        package='controller',
        executable='sabertooth_cmd_vel_bridge',
        name='sabertooth_cmd_vel_bridge',
        output='screen',
    )

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rplidar_ros'), 'launch',
                         'view_rplidar_s3_launch.py')))

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rover_bringup'),
                         'launch', 'realsense_camera.launch.py')))

    wheel_encoder_node = Node(
        package='controller',
        executable='wheel_encoder_node',
        name='wheel_encoder_node',
        output='screen',
    )

    imu_publisher_node = Node(
        package='controller',
        executable='imu_publisher',
        name='imu_publisher',
        output='screen',
    )

    imu_filter_madgwick = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        parameters=[{
            'use_mag': False,
            'publish_tf': False,
            'world_frame': 'enu',
            'base_frame': 'camera_link'
        }],
        remappings=[
            ('/imu/data_raw', '/imu'),
        ],
    )

    odom_publisher_node = Node(
        package='controller',
        executable='odom_publisher',
        name='odom_publisher',
        output='screen',
    )

    return LaunchDescription([
        sabertooth_cmd_vel_bridge_node,
        wheel_encoder_node,
        odom_publisher_node,
        imu_publisher_node,
        rplidar_launch,
        realsense_launch,
        imu_filter_madgwick,
    ])
