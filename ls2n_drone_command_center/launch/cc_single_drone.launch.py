from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("drone_namespace", default_value="Drone1",
                              description="Drone namespace"),
        DeclareLaunchArgument("trajectory", default_value="single_drone_hovering",
                              description="Trajectory to play"),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            [get_package_share_directory('ls2n_drone_command_center'), '/mocap.launch.py'])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('ls2n_drone_command_center'),
                                           '/joystick.launch.py']),
            launch_arguments={'joy_namespace': LaunchConfiguration('drone_namespace')}.items()
        ),
        Node(package='ls2n_drone_command_center', executable='trajectory_publisher',
             output='screen', namespace=LaunchConfiguration('drone_namespace'),
             parameters=[{"trajectory": LaunchConfiguration('trajectory')}]),
    ])
