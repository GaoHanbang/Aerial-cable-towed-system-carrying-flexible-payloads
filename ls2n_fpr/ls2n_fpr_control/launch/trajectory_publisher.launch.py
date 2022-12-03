from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='ls2n_fpr_control', executable='trajectory_publisher_node.py', output='screen',
             parameters=[
                 {'trajectory_mode': 'autopilot'},
                 {'trajectory_file': 'fpr_hovering'}
             ],
             namespace='CommandCenter'
             ),
        Node(package='rqt_gui', executable='rqt_gui',
             output='screen', arguments=["--perspective-file",
                    get_package_share_directory('ls2n_rqt_command_center')+'/resource/LS2N_UAVs_sitl.perspective']),
    ])
