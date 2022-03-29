#!/bin/bash
HOSTS=("192.168.1.100"  "192.168.1.102" )
USERNAMES=("razer"  "oarbot_blue"  )
PASSWORDS=("1234"  "1234")

SCRIPTS=("cd ~/catkin_ws_safe_energetics; 
          git reset --hard; git pull;
          cd ~/catkin_ws_safe_energetics/src/kinova-ros;
          git reset --hard; git pull;"

          "cd ~/catkin_ws_safe_energetics; 
          git reset --hard; git pull;
          cd ~/catkin_ws_safe_energetics/src/kinova-ros;
          git reset --hard; git pull;")
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