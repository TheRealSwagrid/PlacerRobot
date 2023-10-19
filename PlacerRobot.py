#!/usr/bin/env python
import signal
import sys
import time
from copy import copy
from time import sleep

from AbstractVirtualCapability import AbstractVirtualCapability, VirtualCapabilityServer, formatPrint


class PlacerRobot(AbstractVirtualCapability):
    def __init__(self, server):
        super().__init__(server)
        self.uri = f"PlacerRobot"
        self.direction = [0., 1., 0.]
        self.position = [0., 0., 0.]
        self.rotation = [0., 0., 0., 1.]
        self.functionality = {"set_pos_viz": None, "get_name": None, "set_name": None, "get_pos": None, "get_rot": None, "set_rot": None}
        self.current_block_id = None

    def MoveBy(self, params: dict):
        formatPrint(self, f"Forwarding with {params}")
        val = params["SimpleDoubleParameter"]
        if self.functionality["set_name"] is not None:
            self.position = self.functionality["get_pos"]()
        goal = copy(self.position)
        goal[1] += val

        if self.functionality["set_pos_viz"] is not None:
            self.position = self.functionality["set_pos_viz"](goal)
        return {"Position3D": self.position}

    def PlaceBlock(self, params: dict):
        if self.current_block_id is not None:
            self.invoke_sync("detach_block", {"SimpleIntegerParameter": self.current_block_id})
        return {}

    def TransferBlock(self, params: dict):
        self.current_block_id = params["SimpleIntegerParameter"]
        self.invoke_sync("attach_block", {"SimpleIntegerParameter": self.current_block_id,
                                          "SimpleStringParameter": self.functionality["get_name"]()})
        return params

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
