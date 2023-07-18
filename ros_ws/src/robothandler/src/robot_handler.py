#!/usr/bin/env python

import rospy
import tf
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_about_axis
from AbstractVirtualCapability import VirtualCapabilityServer
from visualization_msgs.msg import Marker
from copy import copy
from PlacerRobot import PlacerRobot
from time import sleep


class RobotHandler:
    def __init__(self):
        self.position = np.array([0, 0, 0])
        self.rotation = [0, 0, 0, 1]
        self.scale = .15
        self.br = tf.TransformBroadcaster()
        self.pub = rospy.Publisher("/robot", Marker, queue_size=1)
        self.name = "placerrobot"
        self.max_vel = 0.1
        self.acc = 0.002

    def set_name(self, name: str):
        self.name = name

    def get_tf_name(self):
        return self.name

    def get_pos(self):
        return self.position.tolist()

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

            vel += self.acc
            if vel > 0:
                vel = min(vel, self.max_vel)
            else:
                vel = max(vel, -self.max_vel)

            self.publish_visual()
            sleep((abs(current_vel[0]) + abs(current_vel[1]) + abs(current_vel[2])))

    def publish_visual(self):
        marker = Marker()
        marker.id = 100
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = f"place_robot"
        marker.lifetime = rospy.Duration(0)
        # marker.color.r = .1
        # marker.color.g = .15
        # marker.color.b = .3
        marker.mesh_use_embedded_materials = True
        marker.pose.position.x = self.position[0]
        marker.pose.position.y = self.position[1]
        marker.pose.position.z = self.position[2]
        marker.pose.orientation.x = self.rotation[0]
        marker.pose.orientation.y = self.rotation[1]
        marker.pose.orientation.z = self.rotation[2]
        marker.pose.orientation.w = self.rotation[3]
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
        pos[2] += .2
        pos[1] += 0.5
        pos[0] += 0
        rot = list(quaternion_about_axis(np.deg2rad(90.), [0,0,1]))
        self.br.sendTransform(pos,
                              rot, rospy.Time.now(), self.name, "world")


if __name__ == '__main__':
    rospy.init_node('rosnode')
    rate = rospy.Rate(25)
    rospy.logwarn("HEYO IM HERE")
    server = VirtualCapabilityServer(int(rospy.get_param('~semantix_port')))
    place_robot = PlacerRobot(server)
    place_robot.start()
    robot = RobotHandler()

    place_robot.funtionality["set_pos_viz"] = robot.set_pos
    place_robot.funtionality["get_pos"] = robot.get_pos

    place_robot.funtionality["set_name"] = robot.set_name
    place_robot.funtionality["get_name"] = robot.get_tf_name

    while not rospy.is_shutdown():
        robot.publish_visual()
        rate.sleep()
