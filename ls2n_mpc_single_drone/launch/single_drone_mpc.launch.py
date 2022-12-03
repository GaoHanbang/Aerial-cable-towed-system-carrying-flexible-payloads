from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('drone_namespace', default_value='Drone6',
                              description='Drone Namespace'),
        DeclareLaunchArgument('trajectory', default_value='single_drone_square',
                              description='Trajectory to follow'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('ls2n_drone_command_center'),
                                           '/mocap.launch.py'])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('ls2n_drone_command_center'),
                                          '/joystick.launch.py']),
            launch_arguments={'joy_namespace': LaunchConfiguration('drone_namespace')}.items(),
        ),
        Node(package='ls2n_mpc_single_drone', executable='drone_mpc_node',
             output='screen', namespace=LaunchConfiguration('drone_namespace'),
             parameters=[{'trajectory_file': LaunchConfiguration('trajectory')}])
    ])
