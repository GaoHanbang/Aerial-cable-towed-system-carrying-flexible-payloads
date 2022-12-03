[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

ls2n_drone_ros2
=====================
ls2n_drone_ros2 is a repository containing ROS2 packages aimed to control or perform a simulation of the drones
available in the LS2N laboratory using ROS2. For simulation, we focus on SITL approaches to facilitate the transition
toward real life experiments. For more details about each package, please have a look in the subfolders README files.

Prerequisites
--------------------------
This environment
requires [ros2 galactic](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html#install-ros-2-packages)
and gazebo to work. This file assumes that the reader is accustomed to ROS2. This procedure has been tested with Ubuntu
20.04 Desktop.

The launch files may use *gnome-terminal* to split the outputs in several terminals (therefore, *gnome-terminal* should
be installed, or some launch files should be modified to be adapted to your specific terminal).

1. Clone this repo in the src folder of you ROS2 workspace. You should clone recursively as it contains submodules. If
   you already cloned it you can still get the submodules using
   ```console
   git submodule init
   git submodule update
    ```
2. Install the following packages: python3-rosdep python3-pip ros-galactic-gazebo-ros-pkgs
   python3-colcon-common-extensions openjdk-8-jdk
3. Install with pip: pyros-genmsg
4. Initialise rosdep
   ```console
   sudo rosdep init
   rosdep update
    ```
5. Install dependencies using rosdep (from your ros2 workspace)
   ```console
   rosdep install --from-paths src --ignore-src -y -r
    ```
6. The following steps are required for the SITL simulation.

   Clone the [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot.git) **outside of your ROS2 workspace**. Then set the
   version to the 1.12.3 release. The compatibility is guaranteed only for this version of the PX4 autopilot.
    ```console
   git checkout v1.12.3
   ```
   and init and update the submodules
   ```console
   git submodule update --init --recursive
   ```
8. For the SITL simulation to work properly, you must set the environment variable PX4_SOURCE_DIR
   ```console
   export PX4_SOURCE_DIR=path_to_px4_folder
   ```
   You should add this command to you .bashrc to make it permanent.
9. Install sdkman
   ```console
   curl -s "https://get.sdkman.io" | bash
   source "$HOME/.sdkman/bin/sdkman-init.sh"
   ```
10. Install gradle
    ```console
    sdk install gradle 6.3
    ```
11. Install Fast-RTPS-Gen
   ```console
    git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git -b v1.0.4 ~/Fast-RTPS-Gen \
    && cd ~/Fast-RTPS-Gen \
    && gradle assemble \
    && sudo env "PATH=$PATH" gradle install
   ```
12. Build the packages using colcon and source the setup script of your workspace. You should build two times to
    generate all the trajectories.

Simple trajectory tracking simulation
---------------------------
Activate the position controller by modifying the *onboard_nodes.yaml* file in the *ls2n_drone_bridge* package:

position_control=true

Build your package and then you can start the simulation using:

```console
ros2 launch ls2n_drone_command_center sitl_cc_single_drone.launch.py
```

You can use the fake joystick in rqt to spin the motors then start the experiment (after selecting /Drone1)

Clean build
-----------------
Some updates (especially with the RTPS protocol) require a clean build. To be sure that you build from scatch, you
should:

- In the PX4 firmware folder, delete the build folder
- In your ROS2 workspace, delete the build and install folders.
