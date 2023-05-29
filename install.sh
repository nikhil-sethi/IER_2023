#!/bin/bash
echo "Starting install..."
sleep 2

# Linux
echo "Installing Linux libraries..."
sudo apt-get install tmux python3.8 python3-tk
echo "Done."

## Ardupilot
echo "Installing Ardupilot..."
cd $HOME
git clone --recursive https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
./waf configure
./waf copter

echo "Done."


# Python
echo "Installing Python libraries.." 
pip3 install mavproxy scipy numpy matplotlib geopy tk dronekit pymavlink json pprint geocoder pexpect
echo "Done."