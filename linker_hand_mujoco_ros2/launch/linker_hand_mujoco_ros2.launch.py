from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='linker_hand_mujoco_ros2',
            executable='linker_hand_mujoco_ros2_node',
            name='linker_hand_mujoco_ros2_node',
            output='screen',
            parameters=[{
                'hand_type': 'right',
                'hand_joint': "L6",
                'topic_hz': 30,
                'is_touch': True,
            }],
        ),
    ])
