[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

ls2n_tools
=====================
ROS2 package with useful tools.

Trajectory tools
---
---
To generate trajectory that can be used by others ls2n packages, please look the README file in the trajectories folder.

To list the trajectories generated use

```console
   ros2 run ls2n_tools list_trajectories
```

To plot one generated trajectory use

```console
   ros2 run ls2n_tools plot_trajectory <trajectory_name>
```

Qualisys bridge
---
---
qualisys_bridge.py is a tool to generate topics using from QTM bodies using the QTM python library. 
The mapping between the qtm name and the topic name can be done in the luanch file.