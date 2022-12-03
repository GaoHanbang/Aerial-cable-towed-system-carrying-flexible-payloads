#!/bin/bash
rosdep update
mkdir -p ~/ros2_ws
mkdir -p ~/ros2_ws/src
if ! grep -Fxq "source /opt/ros/galactic/setup.bash" ~/.bashrc
then
  sed -i '1isource /opt/ros/galactic/setup.bash' ~/.bashrc
fi
if ! grep -Fxq "source ~/ros2_ws/install/setup.bash" ~/.bashrc
then
  sed -i '1isource ~/ros2_ws/install/setup.bash' ~/.bashrc
fi