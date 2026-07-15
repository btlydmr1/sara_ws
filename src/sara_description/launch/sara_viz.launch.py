#!/usr/bin/env python3
"""
SARA görselleştirme launch dosyası.

Başlatır:
  - robot_state_publisher : URDF'i yükler, kanatçık TF'lerini /joint_states'ten üretir
  - sara_viz_bridge       : /rocket/fins -> /joint_states, IMU -> base_link TF
  - rviz2                 : 3B görselleştirme

NOT: MAVROS ve fin_controller'ı bu launch BAŞLATMAZ.
Onları mevcut sara_sim_launch.py ile ayrı terminalde çalıştır:
    Terminal 1:  ros2 launch sara_control sara_sim_launch.py
    Terminal 2:  ros2 launch sara_description sara_viz.launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('sara_description')
    urdf_path = os.path.join(pkg, 'urdf', 'sara_rocket.urdf')
    rviz_path = os.path.join(pkg, 'rviz', 'sara.rviz')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='sara_control',
            executable='sara_viz_bridge',
            name='sara_viz_bridge',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_path],
        ),
    ])
