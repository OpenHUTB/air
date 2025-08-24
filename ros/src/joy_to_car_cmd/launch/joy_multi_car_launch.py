from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy_to_car_cmd',  # Package name
            executable='joy_republisher',  # First executable
            name='joy_republisher'
        ),
        Node(
            package='joy_to_car_cmd',  # Package name
            executable='multi_car_joy_controller',  # Second executable
            name='multi_car_joy_controller'
        )
    ])
