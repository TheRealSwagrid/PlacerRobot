#!/usr/bin/env python
import random
import signal
import sys
import threading
import time

import quaternion
from math import sqrt, cos, sin, acos
from copy import copy
from time import sleep

import numpy as np

from AbstractVirtualCapability import AbstractVirtualCapability, VirtualCapabilityServer, formatPrint


class PlacerRobot(AbstractVirtualCapability):
    def __init__(self, server):
        super().__init__(server)
        self.uri = f"PlacerRobot"
        self.use_battery = True
        self.direction = [1., 0., 0.]
        self.position = [0., 0., 0.]
        # x, y, z, w
        self.rotation = [0., 0., 0., 1.]
        self.functionality = {"get_name": None, "set_name": None, "get_pos": None, "set_pos": None, "get_rot": None,
                              "set_rot": None, "rotate": None, "place_block": None, "remove_tf": None}
        self.current_block_id = -1
        self.battery_charge_level = random.uniform(20.0, 100.0) if self.use_battery else 100.
        self.timer = None

    def MoveBy(self, params: dict):
        if self.battery_charge_level == 0.0:
            raise Exception("No battery")
        formatPrint(self, f"Forwarding with {params}")
        val = params["SimpleDoubleParameter"]
        if self.functionality["set_name"] is not None:
            self.position = self.functionality["get_pos"]()

        goal = np.array(self.position)

        goal += val * np.array(self.GetAbsoluteDirection(params)["Vector3"])

        if self.functionality["set_pos"] is not None:
            self.position = self.functionality["set_pos"](goal)
        return {"Position3D": self.position}

    def RotateAroundAxis(self, params: dict):
        if self.battery_charge_level == 0.0:
            raise Exception("No battery")
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
        if self.battery_charge_level == 0.0:
            raise Exception("No battery")
        pos = params["Position3D"]
        if self.current_block_id is not None:
            try:
                if self.functionality["place_block"] is not None:
                    self.functionality["place_block"](pos)
            except Exception as e:
                raise Exception(e)
            # Wait until the block has been set with the accurate position (BlockHandler is slow)
            sleep(.5)
            self.invoke_sync("detach_block", {"SimpleIntegerParameter": self.current_block_id})
            if self.functionality["remove_tf"] is not None:
                self.functionality["remove_tf"]()
            self.current_block_id = -1
        else:
            raise Exception("No Block found")
        return {}

    def TransferBlock(self, params: dict):
        if self.battery_charge_level == 0.0:
            raise Exception("No battery")
        block_id = params["SimpleIntegerParameter"]
        if self.current_block_id != -1 and block_id != -1:
            raise ValueError(f"Still got the Block {self.current_block_id} while waiting for block {block_id}")
        if block_id == -1:
            self.current_block_id = -1
            return params
        self.current_block_id = block_id
        self.invoke_sync("attach_block", {"SimpleIntegerParameter": self.current_block_id,
                                          "SimpleStringParameter": self.functionality["get_name"]()})
        return params


    def SetPosition(self, params: dict):
        if self.battery_charge_level == 0.0:
            raise Exception("No battery")
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
        if self.battery_charge_level == 0.0:
            raise Exception("No battery")
        if len(params["Quaternion"]) != 4:
            raise ValueError(f"Quaternion is WRONG: {params}")
        quat = params["Quaternion"]
        if self.functionality["set_rot"] is not None:
            self.functionality["set_rot"](quat)
            self.rotation = quat
        return {"Quaternion": quat}

    def GetRotation(self, params: dict):
        quat = [0, 0, 0, 0]
        if self.functionality["get_rot"] is not None:
            quat = self.functionality["get_rot"]()
            self.rotation = quat
        return {"Quaternion": quat}

    def GetDirection(self, params: dict):
        return {"Vector3": self.direction}

    def GetAbsoluteDirection(self, params: dict):
        print(f"{self.rotation} - {self.direction}")
        rot = quaternion.as_quat_array(self.rotation)
        new_dir = quaternion.rotate_vectors(rot, np.array(self.direction))
        abs_dir = [1. if x > 0.0 or x < 0.0 else 0. for x in np.abs(new_dir)]
        return {
            "Vector3": abs_dir}

    def SetDirection(self, params: dict):
        new_direction = params["Vector3"]
        self.direction = new_direction
        return self.GetDirection(params)

    def GetBlock(self, params: dict):
        return {"SimpleIntegerParameter": self.current_block_id if self.current_block_id is not None else -1}

    def SetBlock(self, params: dict):
        return self.TransferBlock(params)

    def GetBatteryChargeLevel(self, params: dict):
        return {"BatteryChargeLevel": self.battery_charge_level}

    def SetBatteryChargeLevel(self, params: dict):
        self.battery_charge_level = params["BatteryChargeLevel"]
        return self.GetBatteryChargeLevel(params)

    def loop(self):
        if self.use_battery:
            if self.timer is None:
                self.timer = time.time()
            elif time.time() - self.timer > 5:
                self.timer = time.time()
                self.battery_charge_level -= random.uniform(0.1, 5.0)
                if self.battery_charge_level <= 0.0:
                    self.battery_charge_level = 0.0

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
        listener.uri = "PlacerRobot"
        listener.start()
        signal.signal(signal.SIGTERM, handler)
        listener.join()
    # Needed for properly closing, when program is being stopped wit a Keyboard Interrupt
    except KeyboardInterrupt:
        print("[Main] Received KeyboardInterrupt")
        server.kill()
        listener.kill()
