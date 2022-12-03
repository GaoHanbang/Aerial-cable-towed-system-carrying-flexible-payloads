# Commands for Executing conversion
erb cable_v2.erb > cable.sdf
ros2 service call gazebo/delete_model '{model_name: cable}'
ros2 launch l2sn_drones_cable sitl_drone_bridge.launch.py