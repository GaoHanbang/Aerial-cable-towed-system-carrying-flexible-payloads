#!/bin/bash
if [ $# -ne 3 ]
then
  echo "Usage is  ./install_onboard.sh drone_ip user password"
  exit 1
fi
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
PACKAGE_DIR=$( cd $SCRIPT_DIR/.. &> /dev/null && pwd )
ONBOARD_DIR=/home/$2/ros2_ws/src
DRONE_NAME=$(sshpass -p "$3" ssh "$2"@"$1" "cat /etc/hostname")
sshpass -p "$3" ssh "$2"@"$1" "mkdir -p $ONBOARD_DIR"
sshpass -p "$3" scp -r "$PACKAGE_DIR"  "$2"@"$1":"$ONBOARD_DIR"
sshpass -p "$3" scp "$SCRIPT_DIR/onboard_package.xml"  "$2"@"$1":"$ONBOARD_DIR/ls2n_drone_bridge"/package.xml
sshpass -p "$3" scp "$SCRIPT_DIR/onboard_setup.py"  "$2"@"$1":"$ONBOARD_DIR/ls2n_drone_bridge"/setup.py

sshpass -p "$3" scp $SCRIPT_DIR/config/"$DRONE_NAME"_onboard_params.yaml  "$2"@"$1":"$ONBOARD_DIR/ls2n_drone_bridge"/config/onboard_params.yaml
