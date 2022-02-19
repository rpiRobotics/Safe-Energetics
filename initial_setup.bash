#!/bin/bash
cd; 
mkdir catkin_ws_safe_energetics; 
cd catkin_ws_safe_energetics; 
rm -rf {*,.*};

git clone https://github.com/rpiRobotics/Safe-Energetics.git .;

git clone -b melodic-devel https://github.com/rpiRobotics/fanuc src/testbed/fanuc
# check build dependencies. Note: this may install additional packages,
# depending on the software installed on the machine
$ rosdep update
# be sure to change 'melodic' to whichever ROS release you are using
$ rosdep install --from-paths src/ --ignore-src --rosdistro melodic

##############
source /opt/ros/melodic/setup.bash;
# catkin_make -DCATKIN_BLACKLIST_PACKAGES='...';
# catkin_make -DCATKIN_WHITELIST_PACKAGES='...';
catkin_make
grep -qxF 'source ~/catkin_ws_safe_energetics/devel/setup.bash' ~/.bashrc || echo 'source ~/catkin_ws_safe_energetics/devel/setup.bash' >> ~/.bashrc;
grep -qxF 'export ROSLAUNCH_SSH_UNKNOWN=1' ~/.bashrc || echo 'export ROSLAUNCH_SSH_UNKNOWN=1' >> ~/.bashrc;
source ~/.bashrc;
source ~/catkin_ws_safe_energetics/devel/setup.bash;