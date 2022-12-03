from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory
import yaml
import os
import socket

namespace = socket.gethostname().capitalize()
params = RewrittenYaml(
    source_file=os.path.join(get_package_share_directory('ls2n_drone_bridge'),
                             'config', 'onboard_params.yaml'),
    root_key=namespace,
    param_rewrites={}
)


def generate_launch_description():
    launch_commands = [Node(package='ls2n_px4_ros_com',
                            executable='micrortps_agent',
                            output='screen',
                            namespace=namespace,
                            parameters=[params]),
                       Node(package='ls2n_drone_bridge',
                            executable='drone_bridge',
                            output='screen',
                            namespace=namespace,
                            parameters=[params])]
    with open(os.path.join(get_package_share_directory('ls2n_drone_bridge'),
                           'config', 'onboard_nodes.yaml'), 'r') as stream:
        add_nodes = yaml.safe_load(stream)
        for node in add_nodes:
            if add_nodes[node]:
                launch_commands += [IncludeLaunchDescription(PythonLaunchDescriptionSource(
                    [get_package_share_directory('ls2n_drone_onboard'), '/' + node + '.launch.py']))]
    return LaunchDescription(launch_commands)
