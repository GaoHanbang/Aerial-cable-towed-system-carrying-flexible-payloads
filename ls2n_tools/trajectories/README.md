Trajectories definition files
===
The files in this folder are made to define trajectories that will be automatically generated when the package is built.
The trajectories can have a unlimited number of coordinates. The trajectory file uses the following convention.

File name
---
Can be any name (without space) with .traj name extension.

Header
---
The header of the file contains information about the trajectory filed are implemented like follows

description: trajectory for a single drone identification procedure\
interpolation: polynomial-5\
sampling: 0.005

**interpolation** can be either spline, polynomial, polynomial-5, polynomial-7 or none.

- polynomial is using Krogh polynomial interpolation assuming zero velocity and acceleration at the beginning and the
  end of the trajectory. The specifications of waypoint derivatives are not taken into account.
- spline is a spline interpolation assuming zero velocity and acceleration at the beginning and the end of the
  trajectory. The specifications of waypoint derivatives are not taken into account.
- polynomial-5 and polynomial-7 methods also use Krogh polynomial interpolation, but with the derivatives constraints
  defined at each waypoint, if specified in the trajectory file (otherwise, zero is taken by default). Velocity,
  acceleration (reps. and jerk) can be defined for polynomial-5 (resp. polynomial-7.)

**sampling** only works if interpolation is not none.

A trajectory with no interpolation will be simply published as setpoints at the times defined in the .traj file.

Content
---
A first line with "time" and the names of the coordinates separated by tabulations. Then lines containing the time and
the coordinate values as follows. You can use D (velocity), or DD (acceleration), and DDD (jerk) to specify those values
at waypoint. Indeed, you must not have base variables ending with D.

<pre>
time    x       y       z       yaw       xD
0.0     0.0     0.0     0.0     0.0       0.0
4.0     0.0     0.0     1.0     1.570     0.1
8.0     1.5     0.0     1.5     0.0       0.1
12.0    0.0     -1.5    0.5     -1.570    0.0
16.0    1.0     1.0     1.5     0.0       0.05
20.0    0.0     0.0     0.0     0.0       0.0
</pre>
The file should not contain empty lines. The variables should not finish by a "D" (this letter is used to mark the derivatives only).
