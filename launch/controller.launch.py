#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Args
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    log_level = DeclareLaunchArgument('log_level', default_value='info')
    start_encoder = DeclareLaunchArgument('start_encoder',
                                          default_value='true')
    start_odom = DeclareLaunchArgument('start_odom', default_value='true')
    start_imu = DeclareLaunchArgument('start_imu', default_value='true')
    start_motor = DeclareLaunchArgument('start_motor', default_value='true')

    # Nodes
    encoder_node = Node(package='controller',
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
                        respawn=True,
                        condition=IfCondition(
                            LaunchConfiguration('start_encoder')))

    wheel_odom_node = Node(package='controller',
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
                           respawn=True,
                           condition=IfCondition(
                               LaunchConfiguration('start_odom')))

    imu_node = Node(package='controller',
                    executable='imu_publisher',
                    name='imu_publisher',
                    output='screen',
                    parameters=[{
                        'use_sim_time':
                        LaunchConfiguration('use_sim_time')
                    }],
                    arguments=[
                        '--ros-args', '--log-level',
                        LaunchConfiguration('log_level')
                    ],
                    respawn=True,
                    condition=IfCondition(LaunchConfiguration('start_imu')))

    motor_bridge_node = Node(package='controller',
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
                             respawn=True,
                             condition=IfCondition(
                                 LaunchConfiguration('start_motor')))

    return LaunchDescription([
        use_sim_time,
        log_level,
        start_encoder,
        start_odom,
        start_imu,
        start_motor,
        encoder_node,
        wheel_odom_node,
        # imu_node,
        motor_bridge_node,
    ])
