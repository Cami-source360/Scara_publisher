#!/usr/bin/env python3

"""
Launch file for SCARA Publisher package.

This launch file starts both the SCARA publisher and controller nodes
with configurable parameters.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for SCARA publisher package."""
    
    # Declare launch arguments
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Rate at which to publish joint states (Hz)'
    )
    
    control_rate_arg = DeclareLaunchArgument(
        'control_rate',
        default_value='20.0',
        description='Rate for control loop (Hz)'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('scara_publisher'),
            'config',
            'scara_config.yaml'
        ]),
        description='Path to configuration file'
    )
    
    # SCARA Publisher Node
    scara_publisher_node = Node(
        package='scara_publisher',
        executable='scara_publisher_node',
        name='scara_publisher',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'publish_rate': LaunchConfiguration('publish_rate'),
            }
        ],
        output='screen',
        emulate_tty=True,
    )
    
    # SCARA Controller Node
    scara_controller_node = Node(
        package='scara_publisher',
        executable='scara_controller_node',
        name='scara_controller',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'control_rate': LaunchConfiguration('control_rate'),
            }
        ],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        publish_rate_arg,
        control_rate_arg,
        config_file_arg,
        scara_publisher_node,
        scara_controller_node,
    ])