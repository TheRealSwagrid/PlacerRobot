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
        self.position = [0., 0., 0.]
        self.funtionality = {"set_pos_viz": None, "get_name": None, "set_name": None, "get_pos": None}
        self.current_block_id = None

    def MoveBy(self, params: dict):
        formatPrint(self, f"Forwarding with {params}")
        val = params["SimpleDoubleParameter"]
        goal = copy(self.position)
        goal[1] += val

        if self.funtionality["set_pos_viz"] is not None:
            self.position = self.funtionality["set_pos_viz"](goal)
        return {"Position3D": self.position}

    def PlaceBlock(self, params: dict):

        return params

    def GetPosition(self, params: dict):
        formatPrint(self, f"Get Position {params}")
        if self.funtionality["set_name"] is not None:
            self.position = self.funtionality["get_pos"]()
        return {"Position3D": self.position}

    def Settf_name(self, params: dict):
        tf_name = params["SimpleStringParameter"]
        if self.funtionality["set_name"] is not None:
            self.position = self.funtionality["set_name"](tf_name)
        return {"SimpleStringParameter": tf_name}

    def Gettf_name(self, params: dict):
        tf_name = "NO_ROS_CONNECTION"
        if self.funtionality["get_name"] is not None:
            tf_name = self.funtionality["get_name"]()
        return {"SimpleStringParameter": tf_name}

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
