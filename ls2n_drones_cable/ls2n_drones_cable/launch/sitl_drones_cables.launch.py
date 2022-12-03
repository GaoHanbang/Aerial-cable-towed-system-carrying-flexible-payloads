from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from jinja2 import Environment, FileSystemLoader
from math import pi, cos, sin


def generate_model_file(context, *args, **kwargs):
    # Generate the appropriate template (drones count, cable length)
    model_dir = os.path.join(get_package_share_directory('ls2n_drones_cable'), 'models', 'cable')
    j2_env = Environment(loader=FileSystemLoader([model_dir]))
    template = j2_env.get_template('cable.sdf')
    with open(os.path.join(model_dir, 'cable.sdf'), "w") as f:
        f.write(template.render(
            segment_count=10,
            segment_mass=0.1,
            segment_length=0.1,
            attach_cable_length=0.5,
            attach_cable_mass=0.01))



def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=generate_model_file),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('ls2n_drone_bridge'),
                                           '/sitl_drone_bridge.launch.py']),
            launch_arguments={'drone_model': 'crazy2fly',
                              'number_of_drones': '2',
                              'gz_world': os.path.join(
                                  get_package_share_directory('ls2n_drones_cable'),
                                  'worlds', 'drone_cables.world'),
                              'gz_model_path': os.path.join(
                                  get_package_share_directory('ls2n_drones_cable'), 'models'),
                              'gz_plugin_path': os.path.join(
                                  get_package_share_directory('ls2n_drones_cable_gz_plugins'), 'gz_plugins')
                              }.items()
        ),
        Node(package='rqt_gui', executable='rqt_gui',
             output='screen', arguments=["--perspective-file",
                                         get_package_share_directory(
                                             'ls2n_rqt_command_center') + '/resource/LS2N_UAVs_sitl.perspective']),
        Node(package='ls2n_drone_command_center',
             executable='multi_control',
             namespace='CommandCenter',
             output='screen',
             parameters=[{
                 'drones_to_control': ['Drone1', 'Drone2']
             }]),
        Node(package='ls2n_drones_cable',
             executable='drones_cable',
             namespace='CommandCenter',
             output='screen')
    ])

