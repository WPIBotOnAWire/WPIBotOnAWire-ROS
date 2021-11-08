FROM ros:noetic-robot

ARG USER_ID
ARG GROUP_ID

RUN addgroup --gid $GROUP_ID ros_group
RUN adduser --disabled-password --gecos '' --uid $USER_ID --gid $GROUP_ID ros_runner

RUN apt-get update
RUN apt-get install ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-rosserial python3-pip -y
RUN pip3 install flask requests pyserial
EXPOSE 8080:8080

USER ros_runner
WORKDIR /home/ros_runner/catkin_ws
RUN . /opt/ros/noetic/setup.sh
ENTRYPOINT [ "bash", "-c", "source /opt/ros/noetic/setup.sh && roscore" ]
