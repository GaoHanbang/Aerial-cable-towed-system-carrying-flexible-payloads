from launch import LaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
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
        DeclareLaunchArgument("joystick", default_value="fake",
                              description="Type of joystick (fake/real)", choices=['fake', 'real']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('ls2n_drone_bridge'),
                                           '/sitl_drone_bridge.launch.py']),
            launch_arguments={'drone_model': 'crazy2fly',
                              'number_of_drones': '1'}.items()
        ),
        Node(package='rqt_gui', executable='rqt_gui',
             output='screen', arguments=["--perspective-file",
                    get_package_share_directory('ls2n_rqt_command_center')+'/resource/LS2N_UAVs_sitl.perspective'],
             condition=LaunchConfigurationEquals('joystick', 'fake')),
        Node(package='rqt_gui', executable='rqt_gui',
             output='screen', arguments=["--perspective-file",
                            get_package_share_directory('ls2n_rqt_command_center')+'/resource/LS2N_UAVs.perspective'],
             condition=LaunchConfigurationEquals('joystick', 'real')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('ls2n_drone_command_center'),
                                           '/joystick.launch.py']),
            launch_arguments={'joy_namespace': LaunchConfiguration('drone_namespace')}.items(),
            condition=LaunchConfigurationEquals('joystick', 'real')
        ),
        Node(package='ls2n_drone_command_center', executable='trajectory_publisher',
             output='screen', namespace=LaunchConfiguration('drone_namespace'),
             parameters=[{"trajectory": LaunchConfiguration('trajectory')}]),
    ])
