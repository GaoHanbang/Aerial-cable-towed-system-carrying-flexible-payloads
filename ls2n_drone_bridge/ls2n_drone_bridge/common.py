import numpy as np
from enum import IntEnum
from ls2n_interfaces.srv import *
from ls2n_interfaces.msg import *
from rclpy.qos import qos_profile_sensor_data

qos_profile_sensor_data.depth = 1
qos_profile_px4_com = qos_profile_sensor_data


class FullState:
    sec = 0
    nanosec = 0
    position = np.array([0.0, 0.0, 0.0])
    velocity = np.array([0.0, 0.0, 0.0])
    acceleration = np.array([0.0, 0.0, 0.0])
    orientation = np.array(([1.0, 0.0, 0.0, 0.0]))
    angular_velocity = np.array([0.0, 0.0, 0.0])
    angular_acceleration = np.array([0.0, 0.0, 0.0])
    yaw = 0.0


class ControllerType(IntEnum):
    NONE = 0,
    MOTOR_CONTROL = 1,
    ACCELERATION = 2,
    VELOCITY = 3,
    POSITION = 4,
    ATTITUDE_THRUST = 5,
    RATES_THRUST = 6


DroneStatusString = [None] * (DroneStatus.LAST + 1)
drone_status_dic = DroneStatus.__prepare__(None, None)
for key, value in drone_status_dic.items():
    DroneStatusString[value] = key

DroneRequestString = [None] * (DroneRequest.Request.LAST + 1)
drones_request_dic = DroneRequest.Request.__prepare__(None, None)
for key, value in drones_request_dic.items():
    DroneRequestString[value] = key
