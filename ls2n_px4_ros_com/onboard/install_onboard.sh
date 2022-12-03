#!/bin/bash
if [ $# -ne 3 ]
then
  echo "Usage is  ./install_onboard.sh drone_ip user password"
  exit 1
fi
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
PACKAGE_DIR=$( cd $SCRIPT_DIR/.. &> /dev/null && pwd )
ONBOARD_DIR=/home/$2/ros2_ws/src
LOCAL_INSTALL_DIR=$( ros2 pkg prefix ls2n_px4_ros_com )
LOCAL_BUILD_DIR=$( cd $LOCAL_INSTALL_DIR/../../build/ls2n_px4_ros_com &> /dev/null && pwd )

sshpass -p "$3" ssh "$2"@"$1" "mkdir -p $ONBOARD_DIR"
sshpass -p "$3" scp -r "$PACKAGE_DIR"  "$2"@"$1":"$ONBOARD_DIR"
sshpass -p "$3" scp "$SCRIPT_DIR/onboard_CMakeLists.txt"  "$2"@"$1":"$ONBOARD_DIR/ls2n_px4_ros_com"/CMakeLists.txt
sshpass -p "$3" scp $LOCAL_BUILD_DIR/deserialize_wrapper.h $LOCAL_BUILD_DIR/deserialize_wrapper.cpp $LOCAL_BUILD_DIR/microRTPS_agent.cpp $LOCAL_BUILD_DIR/microRTPS_agent.h "$2"@"$1":"$ONBOARD_DIR/ls2n_px4_ros_com/src"
