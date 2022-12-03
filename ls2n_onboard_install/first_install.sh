#!/bin/bash
######
# This script has been tested for the PX4 VISION with UP CORE computer with ubuntu server 20.04
# It might need to be adapted to your hardware configuration
######

USER=drone
PASSWORD=drone

if [ $# -ne 1 ]
then
  echo "Usage is  ./first_install.sh drone_ip"
  exit 1
fi

sshpass -p "$PASSWORD" ssh "$USER"@"$1" "echo $PASSWORD | sudo -S apt install python3-pip setserial -y"
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
sshpass -p "$PASSWORD" scp "$SCRIPT_DIR/ros2_install.sh" "$SCRIPT_DIR/ros2_configure.sh" "$SCRIPT_DIR/serial_configure.sh" "$USER"@"$1":~
sshpass -p "$PASSWORD" ssh "$USER"@"$1" "echo $PASSWORD | sudo -S chmod +x ros2_install.sh ros2_configure.sh serial_configure.sh"
sshpass -p "$PASSWORD" ssh "$USER"@"$1" "echo $PASSWORD | sudo -S ./ros2_install.sh"
sshpass -p "$PASSWORD" ssh "$USER"@"$1" "./ros2_configure.sh"
# Configuration of the serial port
# sshpass -p "$PASSWORD" ssh "$USER"@"$1" "echo $PASSWORD | sudo -S usermod -a -G dialout $USER"
# sshpass -p "$PASSWORD" ssh "$USER"@"$1" "./serial_configure.sh"

"$SCRIPT_DIR"/install_all_packages.sh $1 $USER $PASSWORD

echo "To finish the installation, please ssh to drone and type:"
echo "cd ros2_ws"
echo "rosdep install --from-paths src --ignore-src -r -y"
echo "colcon build"
