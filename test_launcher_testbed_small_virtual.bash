#!/bin/bash
sleep 1s;

# gnome-terminal --tab --title="ROSCORE" --command "bash -c \"source ~/.bashrc; roscore; exec bash\"";
# sleep 4s;

gnome-terminal --tab --title="TFs" --command "bash -c \"source ~/.bashrc; roslaunch tf_broadcasters_testbed_small tf_broadcasters_testbed_small.launch; exec bash\"";


gnome-terminal --tab --title="KINOVA MOVEIT" --command "bash -c \"source ~/.bashrc;   roslaunch j2n6s300_moveit_config j2n6s300_virtual_robot_demo.launch; exec bash\"";

sleep 3s;

# gnome-terminal --tab --title="RVIZ" --command "bash -c \"source ~/.bashrc; rosrun rviz rviz -d ~/catkin_ws_safe_energetics/src/testbed_small/config/rviz/testbed_small.rviz; exec bash\"";
# gnome-terminal --tab --title="RVIZ" --command "bash -c \"source ~/.bashrc; rosrun rviz rviz -d ~/testbed_small.rviz; exec bash\"";
