from launch import LaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('multi_control_namespace',
                              default_value='CommandCenter',
                              description='Namespace of multi control node'),
        DeclareLaunchArgument('number_of_drones',
                              default_value='2',
                              description='Number of drones'),
        DeclareLaunchArgument('drones_to_control',
                              default_value="['Drone1','Drone2']",
                              description='Array of drones you want to control'),
        DeclareLaunchArgument("joystick", default_value="fake",
                              description="Type of joystick (fake/real)", choices=['fake', 'real']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('ls2n_drone_bridge'),
                                           '/sitl_drone_bridge.launch.py']),
            launch_arguments={'drone_model': 'crazy2fly',
                              'number_of_drones': LaunchConfiguration('number_of_drones')}.items()
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
            launch_arguments={'joy_namespace': LaunchConfiguration('multi_control_namespace')}.items(),
            condition=LaunchConfigurationEquals('joystick', 'real')
        ),
        Node(package='ls2n_drone_command_center',
             executable='multi_control',
             namespace=LaunchConfiguration('multi_control_namespace'),
             output='screen',
             parameters=[{
                 'drones_to_control': LaunchConfiguration('drones_to_control')
             }])
    ])
