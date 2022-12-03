import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    joy_params = os.path.join(
        os.path.join(get_package_share_directory('ls2n_drone_command_center'),'config'),
        'joy-params.yaml')
    
    joy_node = Node(
            package='joy',
            executable='joy_node',
            output='screen',
            parameters=[joy_params],
            remappings=[("joy","CommandCenter/Joystick")])

    return LaunchDescription([
        DeclareLaunchArgument("joy_namespace", default_value="CommandCenter",
                              description="Joystick namespace"),
        joy_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
            target_action=joy_node,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        )),

        Node(
            package='ls2n_fpr_control', 
            executable='fpr_teleop_node.py', 
            output='screen', 
            parameters=[{'interaction':True}],
            namespace=LaunchConfiguration('joy_namespace')
        ),
    ])