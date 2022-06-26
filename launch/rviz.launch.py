import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_name = 'xi_ip'
    rviz_file_dir = os.path.join(get_package_share_directory('xi_ip'), 'rviz')


    return LaunchDescription([
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(rviz_file_dir, 'camera.rviz')]]
        )
    ])
