from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

param_config = os.path.join(
    get_package_share_directory('ls2n_fpr_control'),
    'config',
    'param_fpr.yaml'
)

traj_config = os.path.join(
    get_package_share_directory('ls2n_fpr_control'),
    'config',
    'traj_fpr.yaml'
)


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('ls2n_drone_bridge'),
                                           '/sitl_drone_bridge.launch.py']),
            launch_arguments={'drone_model': 'crazy2fly',
                              'number_of_drones': '3',
                              'gz_world': os.path.join(
                                  get_package_share_directory('ls2n_fpr_control'),
                                  'worlds', 'fpr_interaction.world'),
                              'gz_model_path': os.path.join(
                                  get_package_share_directory('ls2n_fpr_control'), 'models'),
                              'gz_plugin_path': os.path.join(
                                  get_package_share_directory('ls2n_fpr_gz_plugins'), 'gz_plugins')
                              }.items()
        ),
        Node(package='rqt_gui', executable='rqt_gui',
             output='screen', arguments=["--perspective-file",
                        get_package_share_directory('ls2n_rqt_command_center')+'/resource/LS2N_UAVs_sitl.perspective']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('ls2n_drone_command_center'),
                                           '/joystick.launch.py'])
        ),
        Node(package='ls2n_fpr_control', executable='trajectory_publisher_node.py', output='screen',
             parameters=[traj_config],
             namespace='CommandCenter'
             ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('ls2n_fpr_control'),
                                           '/launch/fpr_teleop.launch.py'])
        ),

        Node(package='ls2n_fpr_control',
             executable='autopilot_fpr',
             output='screen',
             parameters=[param_config,
                         {"enable_rviz": True},
                         {"drone_param.drone0.num": 1},
                         {"drone_param.drone1.num": 2},
                         {"drone_param.drone2.num": 3}],
             namespace='CommandCenter'),
    ])
