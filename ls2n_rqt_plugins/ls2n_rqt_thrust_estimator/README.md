# ls2n_rqt_thrust_estimator

This thrust estimator should be only used when disturbances is not compensated, with position control and no
trajectory (or hovering). It will make a linear interpolation between the battery voltage level and the theoretical
maximum thrust for the drone. Those data can be reported in the drone configuration file.