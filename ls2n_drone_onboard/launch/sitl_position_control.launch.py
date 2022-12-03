from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("drone_namespace", default_value="Drone1",
                              description="Drone namespace"),

        Node(package='ls2n_drone_onboard', executable='position_control',
             output='screen', namespace=LaunchConfiguration('drone_namespace'))
    ])
