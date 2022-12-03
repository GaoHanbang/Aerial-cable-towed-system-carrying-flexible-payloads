from launch import LaunchDescription
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory
import os
import socket

namespace = socket.gethostname().capitalize()
params = RewrittenYaml(
    source_file=os.path.join(get_package_share_directory('ls2n_drone_onboard'),
                             'config', 'velocity_joystick_control_params.yaml'),
    root_key=namespace,
    param_rewrites={}
)


def generate_launch_description():
    return LaunchDescription([
        Node(package='ls2n_drone_onboard',
             executable='velocity_joystick_control',
             output='screen',
             namespace=namespace,
             parameters=[params])
    ])
