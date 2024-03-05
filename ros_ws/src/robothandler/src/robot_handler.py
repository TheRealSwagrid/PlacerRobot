#!/usr/bin/env python
import os
import socket
from copy import copy, deepcopy
from time import sleep

import numpy as np
import rospy
import tf
from AbstractVirtualCapability import VirtualCapabilityServer
from tf.transformations import quaternion_about_axis, quaternion_from_euler
from visualization_msgs.msg import Marker

from PlacerRobot import PlacerRobot


class RobotHandler:
    def __init__(self):
        self.position = np.array([0., 0., 0.])
        self.tf_position = None
        self.rotation = [0, 0, 0, 1]
        self.scale = 1#.15
        self.br = tf.TransformBroadcaster()
        self.pub = rospy.Publisher(f"/robot", Marker, queue_size=1)
        self.name = f"placerrobot@{int(rospy.get_param('~semantix_port'))}"
        self.max_vel = 0.1
        self.acc = 0.0001

    def set_name(self, name: str):
        self.name = name

    def get_tf_name(self):
        return self.name

    def get_pos(self):
        return self.position.tolist()

    def rotate(self, axis, deg):
        axis = np.array(axis)
        theta = np.deg2rad(deg)
        self.rotation = list(quaternion_about_axis(theta, axis))
        return self.rotation


    def set_rot(self, rot):
        self.rotation = rot
        return self.rotation

    def place_block(self, goal: list):
        vel = self.acc
        self.tf_position = deepcopy(self.position)
        self.tf_position[2] += .1
        self.tf_position[1] += .2

        while True:
            goal = np.array(goal)
            vector = goal - self.tf_position

            if np.linalg.norm(vector) < 0.1:
                self.tf_position = goal
                self.publish_visual()
                return self.position.tolist()

            current_vel = vel * vector / np.linalg.norm(vector)
            self.tf_position += current_vel

            self.publish_visual()
            sleep((abs(current_vel[0]) + abs(current_vel[1]) + abs(current_vel[2])))
            vel += self.acc
            vel = min(vel, self.max_vel)

    def remove_tf(self):
        self.tf_position = None

    def set_pos(self, goal: list):

        rospy.logwarn(f"Going to Position: {goal}")

        vel = self.acc

        while True:
            goal = np.array(goal)
            vector = goal - self.position

            if np.linalg.norm(vector) < 0.1:
                self.position = goal
                self.publish_visual()
                return self.position.tolist()

            current_vel = vel * vector / np.linalg.norm(vector)
            self.position += current_vel

            self.publish_visual()
            # TODO wait appropriate
            sleep(np.sum(np.abs(current_vel)) * .1)
            vel += self.acc
            vel = min(vel, self.max_vel)

    def publish_visual(self):
        marker = Marker()
        marker.id = 100
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = self.name
        marker.lifetime = rospy.Duration(0)
        marker.mesh_use_embedded_materials = True
        marker.pose.position.x = self.position[0]
        marker.pose.position.y = self.position[1]
        marker.pose.position.z = self.position[2]
        marker.pose.orientation.w = self.rotation[0]
        marker.pose.orientation.x = self.rotation[1]
        marker.pose.orientation.y = self.rotation[2]
        marker.pose.orientation.z = self.rotation[3]
        # Scale down
        marker.scale.x = self.scale
        marker.scale.y = self.scale
        marker.scale.z = self.scale
        marker.color.a = 1
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.mesh_resource = r"package://robothandler/meshes/robot.dae"
        self.pub.publish(marker)

        pos = copy(self.position)

        pos[2] += .1
        pos[1] += .2
        if self.tf_position is not None:
            pos = self.tf_position

        rot = [self.rotation[1], self.rotation[2], self.rotation[3], self.rotation[0]]
        self.br.sendTransform(pos.tolist(),
                              rot, rospy.Time.now(), self.name, "world")


if __name__ == '__main__':
    rospy.init_node('rosnode', xmlrpc_port=int(os.environ["xmlrpc_port"]), tcpros_port=int(os.environ["tcpros_port"]))
    rate = rospy.Rate(25)
    rospy.logwarn("HEYO IM HERE")
    server = VirtualCapabilityServer(int(rospy.get_param('~semantix_port')), socket.gethostbyname(socket.gethostname()))
    place_robot = PlacerRobot(server)
    place_robot.uri = "PlacerRobot"
    place_robot.start()
    robot = RobotHandler()

    place_robot.functionality["get_pos"] = robot.get_pos
    place_robot.functionality["set_pos"] = robot.set_pos

    place_robot.functionality["set_name"] = robot.set_name
    place_robot.functionality["get_name"] = robot.get_tf_name

    place_robot.functionality["get_rot"] = lambda: robot.rotation
    place_robot.functionality["set_rot"] = robot.set_rot
    place_robot.functionality["rotate"] = robot.rotate
    place_robot.functionality["place_block"] = robot.place_block
    place_robot.functionality["remove_tf"] = robot.remove_tf

    while not rospy.is_shutdown():
        robot.publish_visual()
        rate.sleep()
