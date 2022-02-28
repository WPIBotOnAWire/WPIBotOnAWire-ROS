#! /bin/bash

dockerfile="$HOME/catkin_ws/src/WPIBotOnAWire-ROS/"

function bow-start {
    if [ ! -d "$HOME/catkin_ws/devel" ]
    then
        mkdir ~/catkin_ws/devel
        touch ~/catkin_ws/devel/setup.bash
    fi

    if [[ !($(docker image ls --format '{{.Repository}}') =~ 'botonawire_ros')]]
    then
        (docker build -t botonawire_ros:latest --build-arg USER_ID=$(id -u) --build-arg GROUP_ID=$(id -g) $dockerfile)
    fi

    if [[ !($(docker ps -a --format '{{.Names}}') =~ 'botonawire-ros')]]
    then
        (docker create -v ~/catkin_ws:/home/ros_runner/catkin_ws --device=/dev/ttyUSB0 --name botonawire-ros botonawire_ros)
    fi
    if [[ !($(docker ps --format '{{.Names}}') =~ 'botonawire-ros') ]]
    then
        (docker start botonawire-ros)
    fi
}

function bow-term {
    bow-start
    docker exec -it botonawire-ros bash -c 'source ~/catkin_ws/devel/setup.bash && bash'
}

function bow-node {
    bow-start
    docker exec -it botonawire-ros bash -c "source ~/catkin_ws/devel/setup.bash && rosrun $1 $2"
}

function bow-make {
    bow-start
    docker exec -it botonawire-ros bash -c "source /opt/ros/noetic/setup.sh && cd /home/ros_runner/catkin_ws && catkin_make"
}

function bow-echo {
    bow-start
    docker exec -it botonawire-ros bash -c "source /opt/ros/noetic/setup.sh && cd /home/ros_runner/catkin_ws && rostopic echo $argv[1]"
}

function bow-stop {
    docker stop botonawire-ros
}
