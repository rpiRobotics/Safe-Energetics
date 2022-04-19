#!/bin/bash

cd; 

mkdir catkin_ws_safe_energetics; 
cd catkin_ws_safe_energetics; 
rm -rf {*,.*};

git clone https://github.com/rpiRobotics/Safe-Energetics.git .;

cd ~/catkin_ws_safe_energetics;
##############
# source /opt/ros/melodic/setup.bash;
source /opt/ros/noetic/setup.bash;
# catkin_make -DCATKIN_BLACKLIST_PACKAGES='...';
# catkin_make -DCATKIN_WHITELIST_PACKAGES='...';
catkin_make
grep -qxF 'source ~/catkin_ws_safe_energetics/devel/setup.bash' ~/.bashrc || echo 'source ~/catkin_ws_safe_energetics/devel/setup.bash' >> ~/.bashrc;
grep -qxF 'export ROSLAUNCH_SSH_UNKNOWN=1' ~/.bashrc || echo 'export ROSLAUNCH_SSH_UNKNOWN=1' >> ~/.bashrc;
source ~/.bashrc;
source ~/catkin_ws_safe_energetics/devel/setup.bash;