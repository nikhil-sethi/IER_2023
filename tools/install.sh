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

echo -e "
swarm1=28.32652130,77.41895210,0,0\n
swarm2=28.32682130,77.41849156,0,0\n
swarm3=28.32655410,77.41849156,0,0\n
swarm4=28.32682130,77.41895210,0,0\n
swarm5=28.32712130,77.41895210,0,0
" >> ~/ardupilot/Tools/autotest/locations.txt

echo "Done."


# Python
echo "Installing Python libraries.." 
python3 -m pip install empy attrdict3 mavproxy scipy numpy matplotlib geopy tk dronekit pymavlink prettyprint geocoder pexpect
python3 -m pip install -U -f https://extras.wxpython.org/wxPython4/extras/linux/gtk3/ubuntu-22.04 wxPython
echo "Done."