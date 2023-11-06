#!/usr/bin/env python
import signal
import sys
import time
from math import sqrt, cos, sin, acos
from copy import copy
from time import sleep

import numpy as np

from AbstractVirtualCapability import AbstractVirtualCapability, VirtualCapabilityServer, formatPrint


def normalize(v, tolerance=0.00001):
    mag2 = sum(n * n for n in v)
    if abs(mag2 - 1.0) > tolerance:
        mag = sqrt(mag2)
        v = tuple(n / mag for n in v)
    return np.array(v)


class Quaternion:

    def from_axisangle(theta, v):
        v = normalize(v)

        new_quaternion = Quaternion()
        new_quaternion._axisangle_to_q(theta, v)
        return new_quaternion

    def from_value(value):
        new_quaternion = Quaternion()
        new_quaternion._val = value
        return new_quaternion

    def _axisangle_to_q(self, theta, v):
        x = v[0]
        y = v[1]
        z = v[2]

        w = cos(theta / 2.)
        x = x * sin(theta / 2.)
        y = y * sin(theta / 2.)
        z = z * sin(theta / 2.)

        self._val = np.array([x, y, z, w])

    def __mul__(self, b):

        if isinstance(b, Quaternion):
            return self._multiply_with_quaternion(b)
        elif isinstance(b, (list, tuple, np.ndarray)):
            if len(b) != 3:
                raise Exception(f"Input vector has invalid length {len(b)}")
            return self._multiply_with_vector(b)
        else:
            raise Exception(f"Multiplication with unknown type {type(b)}")

    def _multiply_with_quaternion(self, q2):
        x1, y1, z1, w1 = self._val
        x2, y2, z2, w2 = q2._val
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
        z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2

        result = Quaternion.from_value(np.array((x, y, z, w)))
        return result

    def _multiply_with_vector(self, v):
        q2 = Quaternion.from_value(np.append(v, 0.0))
        return (self * q2 * self.get_conjugate())._val[:3]

    def get_conjugate(self):
        x, y, z, w = self._val
        result = Quaternion.from_value(np.array((-x, -y, -z, w)))
        return result

    def __repr__(self):
        v, theta = self.get_axisangle()
        return f"((%.6f; %.6f, %.6f, %.6f))" % (v[0], v[1], v[2], theta)

    def get_axisangle(self):
        v, w = self._val[:3], self._val[-1]
        theta = acos(w) * 2.0

        return normalize(v), theta

    def tolist(self):
        return self._val.tolist()

    def vector_norm(self):
        v, w = self.get_axisangle()
        return np.linalg.norm(v)


class PlacerRobot(AbstractVirtualCapability):
    def __init__(self, server):
        super().__init__(server)
        self.uri = f"PlacerRobot"
        self.direction = [0., 1., 0.]
        self.position = [0., 0., 0.]
        # x, y, z, w
        self.rotation = [0., 0., 0., 1.]
        self.functionality = {"get_name": None, "set_name": None, "get_pos": None, "set_pos": None, "get_rot": None,
                              "set_rot": None, "rotate": None, "place_block": None, "remove_tf": None}
        self.current_block_id = None

    def MoveBy(self, params: dict):
        formatPrint(self, f"Forwarding with {params}")
        val = params["SimpleDoubleParameter"]
        if self.functionality["set_name"] is not None:
            self.position = self.functionality["get_pos"]()

        goal = copy(self.position)


        goal += val * self.GetAbsoluteDirection(params)["Vector3"]


        if self.functionality["set_pos"] is not None:
            self.position = self.functionality["set_pos"](goal)
        return {"Position3D": self.position}

    def RotateAroundAxis(self, params: dict):
        axis = params["Axis"]
        if axis == 'z':
            axis = [0, 0, 1]
        elif axis == 'y':
            axis = [0, 1, 0]
        elif axis == 'x':
            axis = [1, 0, 0]
        degree = params["SimpleDoubleParameter"]
        if self.functionality["get_name"] is not None:
            quat = self.functionality["rotate"](axis, degree)
            self.rotation = quat
        formatPrint(self, f"New Quaternion {quat}")
        return {"Quaternion": quat}

    def PlaceBlock(self, params: dict):
        pos = params["Position3D"]
        if self.current_block_id is not None:
            if self.functionality["place_block"] is not None:
                self.functionality["place_block"](pos)
            self.invoke_sync("detach_block", {"SimpleIntegerParameter": self.current_block_id})
            if self.functionality["remove_tf"] is not None:
                self.functionality["remove_tf"]()
        else:
            raise Exception("No Block found")
        return {}

    def TransferBlock(self, params: dict):
        self.current_block_id = params["SimpleIntegerParameter"]
        self.invoke_sync("attach_block", {"SimpleIntegerParameter": self.current_block_id,
                                          "SimpleStringParameter": self.functionality["get_name"]()})
        return params

    def SetPosition(self, params: dict):
        formatPrint(self, f"Set Position {params}")

        if self.functionality["set_pos"] is not None:
            self.position = self.functionality["set_pos"](params["Position3D"])
        return {"Position3D": self.position}

    def GetPosition(self, params: dict):
        formatPrint(self, f"Get Position {params}")
        if self.functionality["set_name"] is not None:
            self.position = self.functionality["get_pos"]()
        return {"Position3D": self.position}

    def Settf_name(self, params: dict):
        tf_name = params["SimpleStringParameter"]
        if self.functionality["set_name"] is not None:
            self.position = self.functionality["set_name"](tf_name)
        return {"SimpleStringParameter": tf_name}

    def Gettf_name(self, params: dict):
        tf_name = "NO_ROS_CONNECTION"
        if self.functionality["get_name"] is not None:
            tf_name = self.functionality["get_name"]()
        return {"SimpleStringParameter": tf_name}

    def SetRotation(self, params: dict):
        quat = params["Quaternion"]
        if self.functionality["set_rot"] is not None:
            self.functionality["set_rot"](quat)
        return {"Quaternion": quat}

    def GetRotation(self, params: dict):
        quat = [0, 0, 0, 0]
        if self.functionality["get_rot"] is not None:
            quat = self.functionality["get_rot"]()
        return {"Quaternion": quat}

    def GetDirection(self, params: dict):
        return {"Vector3": self.direction}

    def GetAbsoluteDirection(self, params: dict):
        new_dir = np.round(Quaternion.from_value(np.array(self.rotation)) * self.direction, decimals=6)
        norm_dir = new_dir / np.linalg.norm(new_dir)

        return {
            "Vector3": norm_dir}

    def SetDirection(self, params: dict):
        new_direction = params["Vector3"]
        self.direction = new_direction
        return self.GetDirection(params)

    def loop(self):
        pass


if __name__ == '__main__':
    # Needed for properly closing when process is being stopped with SIGTERM signal
    def handler(signum, frame):
        print("[Main] Received SIGTERM signal")
        listener.kill()
        quit(1)


    try:
        port = None
        if len(sys.argv[1:]) > 0:
            port = int(sys.argv[1])
        server = VirtualCapabilityServer(port)
        listener = PlacerRobot(server)
        listener.start()
        signal.signal(signal.SIGTERM, handler)
        listener.join()
    # Needed for properly closing, when program is being stopped wit a Keyboard Interrupt
    except KeyboardInterrupt:
        print("[Main] Received KeyboardInterrupt")
        server.kill()
        listener.kill()
