[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

ls2n_px4_ros_com
=====================
This package is inspired from [px4_ros_com](https://github.com/PX4/px4_ros_com). It is, however encapsulated in a ROS2
node for better compatibility of the ROS2 topics. It will use the current ROS2 DDS protocol instead of fastrtps for the
orginal code. The desired uorb messages from PX4 to be broadcast to ROS can be selected in the *uorb_rtps_message_ids.yaml*
file.

The PX4_SOURCE_DIR environment variable must be set before building. 
```
export PX4_SOURCE_DIR=<path_to_px4_folder> 
```

The building process will automatically updates the
firmware configuration file and rebuild the SITL version. The onboard version must be build/uploaded manually.