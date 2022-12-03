[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

ls2n_drone_onboard
=====================
This package contains nodes for classic embedded applications.

Observers
---------------------
This node performs two observers.

- The disturbance observer computes the disturbances as a force in world frame assuming a rigid body model of the drone
  and the thrusts inputs sent to the autopilot.
- The maximum thrust observer computes the maximum thrust of the drone. This computation is efficient only in near
  hovering situation without disturbances (ex: wind).

The disturbances' observer output can be fed back to the drone bridge for a better behavior of the position / velocity
controllers.

Position control
-----------------------
Position control is an intermediate node to control drone_bridge in position using a trajectory topic.

Velocity joystick control
-----------------------
Position control is an intermediate node to control drone_bridge in velocity using joystick inputs.
topic.
