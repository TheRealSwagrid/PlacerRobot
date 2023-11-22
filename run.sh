#!/usr/bin/env bash

port="$1"
cp PlacerRobot.py ros_ws/src/robothandler/src
cp AbstractVirtualCapability.py ros_ws/src/robothandler/src
cd ros_ws && source /opt/ros/noetic/setup.bash && catkin_make
source devel/setup.bash && roslaunch robothandler robothandler.launch semantix_port:="$port"
