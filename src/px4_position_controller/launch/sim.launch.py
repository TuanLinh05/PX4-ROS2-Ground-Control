from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_position_controller',
            executable='position_controller_node',
            output='screen',
        ),
        Node(
            package='px4_gen_trajectory',
            executable='gen_trajectory_node',
            output='screen',
        ),
        Node(
            package='px4_trajectory_visualizer',
            executable='trajectory_visualizer_node',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
        ),
    ])
