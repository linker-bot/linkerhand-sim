#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='linker_hand_pybullet_ros2',
            executable='linker_hand_pybullet_ros2_node',
            name='linker_hand_pybullet_ros2_node',
            output='screen',
            parameters=[{
                'hand_joint': "L20",
            }],
        ),
    ])
