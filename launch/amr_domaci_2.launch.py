from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() :
    return LaunchDescription([
        Node(
            package='amr_domaci_2',
            executable='talker',
            name='talker',
            output='screen',
        ),
        Node(
            package='amr_domaci_2',
            executable='listener',
            name='lisntene',
            output='screen',
        ),
    ])