# Safe Energetics
This is a repository for Safe Energetics Project


## Setting up the system

Assuming you have ROS Melodic installed on Ubuntu 18.04,

Without cloning this repo by yourself, simply download and run
```
./initial_setup.bash
```

It creates a folder named `catkin_ws_safe_energetics` in your home directory and builds the packages in this repo including the ROS-I FANUC support repo.

## Launching Test Simulation

Run
```
roslaunch safe_energetics_testbed test_safe_energetics_testbed.launch
```

You will be able to adjust the joints of FANUC M710-iC/45M robot in the RPI testbed cage

![test_simulation](/.readme_include/test_simulation.png)

## Launching Test Simulation with MoveIt
Run
```
roslaunch safe_energetics_testbed_moveit_config demo.launch
```
![test_simulation_with_moveit](/.readme_include/test_simulation_with_moveit.png)
