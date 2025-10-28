#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get RealSense config path from controller package
    controller_dir = get_package_share_directory('controller')
    
    # Args
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    log_level = DeclareLaunchArgument('log_level', default_value='info')

    # Nodes
    wheel_encoder_node = Node(package='controller',
                        executable='wheel_encoder',
                        name='wheel_encoder',
                        output='screen',
                        parameters=[{
                            'use_sim_time':
                            LaunchConfiguration('use_sim_time')
                        }],
                        arguments=[
                            '--ros-args', '--log-level',
                            LaunchConfiguration('log_level')
                        ],
                        respawn=True)

    wheel_odom_publisher_node = Node(package='controller',
                           executable='wheel_odom_publisher',
                           name='wheel_odom_publisher',
                           output='screen',
                           parameters=[{
                               'use_sim_time':
                               LaunchConfiguration('use_sim_time')
                           }],
                           arguments=[
                               '--ros-args', '--log-level',
                               LaunchConfiguration('log_level')
                           ],
                           respawn=True)

    sabertooth_cmd_vel_bridge_node = Node(package='controller',
                             executable='sabertooth_cmd_vel_bridge',
                             name='sabertooth_cmd_vel_bridge',
                             output='screen',
                             parameters=[{
                                 'use_sim_time':
                                 LaunchConfiguration('use_sim_time')
                             }],
                             arguments=[
                                 '--ros-args', '--log-level',
                                 LaunchConfiguration('log_level')
                             ],
                             respawn=True)

    return LaunchDescription([
        use_sim_time,
        log_level,
        wheel_encoder_node,
        wheel_odom_publisher_node,
        sabertooth_cmd_vel_bridge_node,
    ])
