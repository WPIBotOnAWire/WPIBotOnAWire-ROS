#! /bin/fish

set dockerfile ~/catkin_ws/src/WPIBotOnAWire/

function bow-start
    if not test -e ~/catkin_ws/devel
        mkdir ~/catkin_ws/devel
        touch ~/catkin_ws/devel/setup.bash
    end

    if not string match -q (docker image ls --format '{{.Repository}}') 'botonawire_ros'
        docker build -t botonawire_ros:latest --build-arg USER_ID=(id -u) --build-arg GROUP_ID=(id -g) $dockerfile
    end

    if not string match -q (docker ps -a --format '{{.Names}}') 'botonawire-ros'
        docker create -v ~/catkin_ws:/home/ros_runner/catkin_ws --device=/dev/ttyUSB0 --name botonawire-ros botonawire_ros
    end

    if not string match -q (docker ps --format '{{.Names}}') 'botonawire-ros'
        docker start botonawire-ros
    end
end

function bow-term
    bow-start
    docker exec -it botonawire-ros bash -c 'source /opt/ros/noetic/setup.sh && source ~/catkin_ws/devel/setup.bash && bash'
end

function bow-node
    bow-start
    docker exec -it botonawire-ros bash -c "source ~/catkin_ws/devel/setup.bash && rosrun $argv[1] $argv[2]"
end

function bow-make
    bow-start
    docker exec -it botonawire-ros bash -c "source /opt/ros/noetic/setup.sh && cd /home/ros_runner/catkin_ws && catkin_make"
end

function bow-echo
    bow-start
    docker exec -it botonawire-ros bash -c "source /opt/ros/noetic/setup.sh && cd /home/ros_runner/catkin_ws && rostopic echo $argv[1]"
end

function bow-stop
    docker stop botonawire-ros
end
