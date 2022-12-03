from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    params = RewrittenYaml(
        source_file=os.path.join(get_package_share_directory('ls2n_drone_onboard'),
                                 'config', 'velocity_joystick_control_params.yaml'),
        root_key=LaunchConfiguration("drone_namespace").perform(context),
        param_rewrites={}
    )
    return [Node(package='ls2n_drone_onboard', executable='velocity_joystick_control',
                 output='screen', namespace=LaunchConfiguration('drone_namespace'),
                 parameters=[params])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("drone_namespace", default_value="Drone1",
                              description="Drone namespace"),
        OpaqueFunction(function=launch_setup)
    ])
