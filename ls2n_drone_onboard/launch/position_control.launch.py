from launch import LaunchDescription
from launch_ros.actions import Node
import socket

namespace = socket.gethostname().capitalize()


def generate_launch_description():
    return LaunchDescription([
        Node(package='ls2n_drone_onboard',
             executable='position_control',
             output='screen',
             namespace=namespace)
    ])
