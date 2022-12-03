std_msgs/Header header
std_msgs/String velocity_mode
# Platform Velocity
geometry_msgs/Vector3 platform_linear_vel
geometry_msgs/Vector3 platform_linear_accel
std_msgs/Float64 platform_yaw_rate
std_msgs/Float64 platform_yaw_accel
# Platform Angles
std_msgs/Float64 platform_roll
std_msgs/Float64 platform_roll_rate
std_msgs/Float64 platform_pitch
std_msgs/Float64 platform_pitch_rate
# Leg Angles
std_msgs/Float64[3] leg_angles
std_msgs/Float64[3] leg_angles_vel