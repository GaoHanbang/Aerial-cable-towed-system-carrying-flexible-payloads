#!/bin/bash

USER=drone
PASSWORD=drone

if [ $# -ne 2 ]
then
  echo "Usage is  ./update_build drone_ip package_name"
  exit 1
fi
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
if test -f $SCRIPT_DIR/../$2/onboard/install_onboard.sh; then
  $SCRIPT_DIR/../$2/onboard/install_onboard.sh $1 $USER $PASSWORD
else
  sshpass -p "$PASSWORD" scp -r "$SCRIPT_DIR/../$2"  "$USER"@"$1":~/ros2_ws/src
fi
sshpass -p "$PASSWORD" ssh "$USER"@"$1" "rm -r ~/ros2_ws/install/$2 ~/ros2_ws/build/$2 && cd ros2_ws && colcon build --packages-select $2"