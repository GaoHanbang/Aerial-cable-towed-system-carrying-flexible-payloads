#!/bin/bash

if [ $# -ne 3 ]
then
  echo "Usage is  ./install_all_packages drone_ip user password"
  exit 1
fi
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
for package in "px4_msgs" "ls2n_interfaces" "ls2n_drone_onboard" "ls2n_drone_bridge" "ls2n_px4_ros_com"
do
  if test -f $SCRIPT_DIR/../$package/onboard/install_onboard.sh; then
      $SCRIPT_DIR/../$package/onboard/install_onboard.sh $1 $2 $3
    else
      sshpass -p "$3" scp -r "$SCRIPT_DIR/../$package"  "$2"@"$1":~/ros2_ws/src
    fi
done
