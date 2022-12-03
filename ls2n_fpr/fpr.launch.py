from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("interaction", default_value="False",
                              description="Enable interaction experiment"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource([get_package_share_directory('ls2n_tools'),
                                                                '/mocap.launch.py'])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('ls2n_fpr_control'),
                                          '/launch/autopilot_fpr.launch.py']),
            launch_arguments={'interaction_experiment': LaunchConfiguration('interaction')}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('ls2n_fpr_control'),
                                          '/launch/fpr_teleop.launch.py']),
            condition=IfCondition(LaunchConfiguration("interaction"))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('ls2n_drone_command_center'),
                                          '/joystick.launch.py']),
            condition=UnlessCondition(LaunchConfiguration("interaction"))
        ),
    ])
