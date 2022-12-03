from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='ls2n_drone_command_center', executable='qualisys', remappings=[
            ("crazy2fly1", "/Drone1/Mocap/odom"),
            ("crazy2fly2", "/Drone2/Mocap/odom"),
            ("crazy2fly3", "/Drone3/Mocap/odom"),
            ("crazy2fly4", "/Drone4/Mocap/odom"),
            ("crazy2fly5", "/Drone5/Mocap/odom"),
            ("crazy2fly6", "/Drone6/Mocap/odom"),
            ("crazy2fly7", "/Drone7/Mocap/odom"),
            ("crazy2fly8", "/Drone8/Mocap/odom"),
            ("drone9", "/Drone9/Mocap/odom"),
            ("platform_fpr", "/FPR/Mocap/odom"),
            ("crazyflie1", "/Drone1/Mocap/odom")],
             parameters=[{'ip_address': '192.168.0.10'}])
    ])
