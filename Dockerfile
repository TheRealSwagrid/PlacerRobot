FROM ros:noetic
SHELL ["/bin/bash", "-c"]

ENV semantix_port=7500
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_IP=127.0.0.1


# ROS-Noetic Setup
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-get update && apt-get install -y curl
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

RUN sudo apt-get update
RUN apt-get update && apt-get install -y python3-rosinstall python3-rosinstall-generator python3-wstool  \
    build-essential python3-rosdep python3-catkin-tools ros-noetic-vrpn-client-ros python-is-python3 python3-pip

# Add Files
ADD ros_ws /ros_ws
COPY protocols /etc
COPY PlacerRobot.py /ros_ws/src/robothandler/src
COPY AbstractVirtualCapability.py ros_ws/src/robothandler/src

# Build Ros-Pkg and build
RUN cd /ros_ws && source /opt/ros/noetic/setup.bash && catkin_make
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD source /ros_ws/devel/setup.bash && roslaunch robothandler robothandler.launch semantix_port:=${semantix_port}
