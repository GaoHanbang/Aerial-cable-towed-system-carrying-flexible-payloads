from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

config = os.path.join(
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
        DeclareLaunchArgument('interaction_experiment', default_value='False',
                              description='Enable interaction experiment'),
        Node(package='ls2n_fpr_control', executable='autopilot_fpr', output='screen',
             parameters=[config,
                         {'enable_rviz': True},
                         {'interaction_experiment': LaunchConfiguration('interaction_experiment')}],
             namespace='CommandCenter'
             ),
        Node(package='ls2n_fpr_control', executable='trajectory_publisher_node.py', output='screen',
             parameters=[traj_config],
             namespace='CommandCenter'
             ),
        Node(package='ls2n_fpr_control', executable='interaction_detector.py', output='screen',
             condition=IfCondition(LaunchConfiguration("interaction"))
             )
    ])
