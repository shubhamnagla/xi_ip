from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xi_ip',
            namespace='image_flter',
            executable='image_filter_cpp',
            name='image_filter'
        )
    ])
