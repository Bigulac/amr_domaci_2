from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() :
    return LaunchDescription([
        Node(
            package='amr_domaci_2',
            executable='drive_to_target_point',
            name='drive_to_target_point',
            output='screen',
        ),
        Node(
            package='amr_domaci_2',
            executable='manual_control',
            name='manual_control',
            output='screen',
            emulate_tty=True,
            prefix='xterm -e',
        ),
        Node(
            package='amr_domaci_2',
            executable='op_mode_service',
            name='op_mode_service',
            output='screen',
        ),
    ])