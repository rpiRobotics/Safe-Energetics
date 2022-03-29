#!/bin/bash
HOSTS=("192.168.1.100"  "192.168.1.102" )
USERNAMES=("razer"  "oarbot_blue"  )
PASSWORDS=("1234"  "1234")

SCRIPTS=(
    ############### RAZER (MAIN COMPUTER) ##############
    "cd; 

    echo 1234 | sudo -S usermod -a -G dialout razer;
    echo 1234 | sudo -S apt install -y curl;
    echo 1234 | sudo -S apt install -y software-properties-common;
    echo 1234 | sudo -S apt install -y ros-melodic-moveit*
    echo 1234 | sudo -S apt install -y python3-pip;
    echo 1234 | sudo -S apt install -y python-pip;
    echo 1234 | sudo wget https://raw.githubusercontent.com/Kinovarobotics/kinova-ros/melodic-devel/kinova_driver/udev/10-kinova-arm.rules;
    echo 1234 | sudo rm 10-kinova-arm.rules.*; # removes the duplicates

    ###########################
    mkdir catkin_ws_safe_energetics; 
    cd catkin_ws_safe_energetics; 
    rm -rf {*,.*};

    git clone https://github.com/rpiRobotics/Safe-Energetics.git .;

    git clone -b melodic-devel https://github.com/rpiRobotics/fanuc src/testbed/fanuc
    # check build dependencies. Note: this may install additional packages,
    # depending on the software installed on the machine
    rosdep update
    # be sure to change 'melodic' to whichever ROS release you are using
    rosdep install --from-paths src/testbed/ --ignore-src --rosdistro melodic

    cd src;
    git clone https://github.com/burakaksoy/kinova-ros.git kinova-ros;

    cd ..;
    ##############
    source /opt/ros/melodic/setup.bash;
    # catkin_make -DCATKIN_BLACKLIST_PACKAGES='...';
    # catkin_make -DCATKIN_WHITELIST_PACKAGES='...';
    catkin_make
    grep -qxF 'source ~/catkin_ws_safe_energetics/devel/setup.bash' ~/.bashrc || echo 'source ~/catkin_ws_safe_energetics/devel/setup.bash' >> ~/.bashrc;
    grep -qxF 'export ROSLAUNCH_SSH_UNKNOWN=1' ~/.bashrc || echo 'export ROSLAUNCH_SSH_UNKNOWN=1' >> ~/.bashrc;
    source ~/.bashrc;
    source ~/catkin_ws_safe_energetics/devel/setup.bash;"

    ############### OARBOT_BLUE ##############
    "cd; 

    echo 1234 | sudo -S usermod -a -G dialout oarbot_blue;
    echo 1234 | sudo -S apt install -y curl;
    echo 1234 | sudo -S apt install -y software-properties-common;
    echo 1234 | sudo -S apt install -y ros-melodic-moveit*
    echo 1234 | sudo -S apt install -y python3-pip;
    echo 1234 | sudo -S apt install -y python-pip;
    echo 1234 | sudo wget https://raw.githubusercontent.com/Kinovarobotics/kinova-ros/melodic-devel/kinova_driver/udev/10-kinova-arm.rules;
    echo 1234 | sudo rm 10-kinova-arm.rules.*; # removes the duplicates

    ###########################
    mkdir catkin_ws_safe_energetics; 
    cd catkin_ws_safe_energetics; 
    rm -rf {*,.*};

    git clone https://github.com/rpiRobotics/Safe-Energetics.git .;

    git clone -b melodic-devel https://github.com/rpiRobotics/fanuc src/testbed/fanuc
    # check build dependencies. Note: this may install additional packages,
    # depending on the software installed on the machine
    rosdep update
    # be sure to change 'melodic' to whichever ROS release you are using
    rosdep install --from-paths src/testbed/ --ignore-src --rosdistro melodic

    cd src;
    git clone https://github.com/burakaksoy/kinova-ros.git kinova-ros;

    cd ..;
    ##############
    source /opt/ros/melodic/setup.bash;
    # catkin_make -DCATKIN_BLACKLIST_PACKAGES='...';
    # catkin_make -DCATKIN_WHITELIST_PACKAGES='...';
    catkin_make
    grep -qxF 'source ~/catkin_ws_safe_energetics/devel/setup.bash' ~/.bashrc || echo 'source ~/catkin_ws_safe_energetics/devel/setup.bash' >> ~/.bashrc;
    grep -qxF 'export ROSLAUNCH_SSH_UNKNOWN=1' ~/.bashrc || echo 'export ROSLAUNCH_SSH_UNKNOWN=1' >> ~/.bashrc;
    source ~/.bashrc;
    source ~/catkin_ws_safe_energetics/devel/setup.bash;")

echo ${SCRIPTS}
for i in ${!HOSTS[*]} ; do
    echo "------------"
    # echo ${HOSTS[i]}
    echo ${USERNAMES[i]}
    # echo ${PASSWORDS[i]}
    echo ${SCRIPTS[i]}
    ssh-keygen -f "$HOME/.ssh/known_hosts" -R ${HOSTS[i]}
    # sudo apt-get install sshpass
    sshpass -p ${PASSWORDS[i]} ssh -t -o StrictHostKeyChecking=no -o HostKeyAlgorithms='ssh-rsa' -o ConnectTimeout=2 -l ${USERNAMES[i]} ${HOSTS[i]} "${SCRIPTS[i]}"
    # ssh -o StrictHostKeyChecking=no -l ${USERNAMES[i]} ${HOSTS[i]} "${SCRIPTS[i]}"
done

# echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc; 