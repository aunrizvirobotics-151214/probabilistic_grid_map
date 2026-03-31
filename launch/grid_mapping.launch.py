"""
grid_mapping launch file
=========================
Starts:
  1. grid_mapping_node  — subscribes /scan + /odom, publishes /grid_map
  2. rviz2              — visualises the growing occupancy grid and laser scan

Usage:
  ros2 launch grid_mapping grid_mapping.launch.py

Override parameters at runtime:
  ros2 launch grid_mapping grid_mapping.launch.py \\
      params_file:=/path/to/my_params.yaml

Override RViz config:
  ros2 launch grid_mapping grid_mapping.launch.py \\
      rviz_config:=/path/to/my_config.rviz
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg = FindPackageShare('grid_mapping')

    # ── Launch arguments ──────────────────────────────────────────────────────
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([pkg, 'config', 'grid_mapping_params.yaml']),
        description='Full path to the grid_mapping parameters YAML file',
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([pkg, 'rviz', 'grid_mapping.rviz']),
        description='Full path to the RViz2 config file',
    )

    # ── 1. Grid mapping node ──────────────────────────────────────────────────
    grid_mapping_node = Node(
        package='grid_mapping',
        executable='grid_mapping_node',
        name='grid_mapping_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    # ── 2. RViz2 ─────────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen',
    )

    return LaunchDescription([
        params_arg,
        rviz_arg,
        grid_mapping_node,
        rviz_node,
    ])
