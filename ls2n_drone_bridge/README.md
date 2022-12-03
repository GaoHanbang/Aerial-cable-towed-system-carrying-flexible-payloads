[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

ls2n_drone_bridge
=====================
LS2N_drone_bridge is a ROS2 package to interact with the drones available at the LS2N laboratory. It is designed to
minimise the data exchange between the autopilot and the onboard computer and keep low response time (compared to
MAVLINK). However, all functionalities are not available.

Supported firmwares
--------------------------
**PX4 V1.12.3** with DDS protocol (micro_rtps agent). For onboard application, a serial link should be established
between the drone and the PX4 Autopilot, and the micro_rtps client module must be active on the drone. Note that some
messages are required for the bridge to work properly.

Usage
---------------------------
This bridge can be used either in SITL simulation or onboard the drone, using the appropriate launch file. The namespace
of the drone will be obtained from the hostname of the companion computer or automatically affected in simulation. To
run the SITL simulation, the environment variable PX4_SOURCE_DIR must be set.

```
export PX4_SOURCE_DIR=<path_to_px4_folder> 
```

Commands can be sent using the *Request* service. The first command will always be spin motors, then the take-off can be
requested, or directly switch to some controllers (Attitude/Thrust, Rates/Thrust). Controllers can be switched in
flight.

Topics
--------------------------
Topics ending with "PubSubTopic" are automatically generated from the rtps agent and are complicated to use as they are
directly translated as a UORB topic for the PX4 controller. It is thus recommended to not use those topics unless you
know exactly what you are doing. You may use following topics and services to interact with the bridge.

- *KeepAlive*: This topic ensures that communication is up with the bridge. Offboard mode is maintained only if the keep
  alive is up. Is no messages are received on this topic for more than 0.1 second, the drone goes in "emergency
  shutdown" mode.
- *Status*: Provides feedback about the drone status.
- *Trajectory*: Publish to this topic when controlling the drone with the position, velocity or acceleration controller.
  The yaw can also be set using this topic.
- *AttitudeThrustSetPoint*: Set points for attitude/thrust control. Attitude in quaternion and thrust from 0 to
  max_thrust.
- *RatesThrustSetPoint*: Set points for rates/thrust control. rates in rad/sec and thrust from 0 to max_thrust.
- *Mocap/odom*: MOCAP informations to be sent to the drone.
- *EKF/odom*: Odometry computed by the drone (with internal EKF).

Services
----------------------------

- *Request*: To send a DroneRequest to the bridge (see service definition for available requests)
