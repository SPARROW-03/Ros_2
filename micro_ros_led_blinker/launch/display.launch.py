from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='micro_ros_led_blinker',
            executable='led_blinker',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        )
    ])
