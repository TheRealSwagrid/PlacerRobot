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
        self.funtionality = {"set_pos_viz": None, "get_name": None, "set_name": None}
        self.max_vel = 0.25
        self.acc = 0.002
        self.current_block_id = None

    def MoveBy(self, params: dict):
        formatPrint(self, f"Forwarding with {params}")
        val = params["SimpleDoubleParameter"]

        goal = copy(self.position)
        goal[1] += val
        #self.funtionality["set_pos_viz"](goal)
        #return {"Position3D": goal}

        current_vel = 0

        formatPrint(self, f"Pos: {self.position} Goal {goal} | {abs(self.position[1] - goal[1])} ")
        while abs(self.position[1] - goal[1]) > 0.1:
            if val > 0:
                current_vel += self.acc
                current_vel = self.max_vel if current_vel > self.max_vel else current_vel
            elif val < 0:
                current_vel -= self.acc
                current_vel = -self.max_vel if -current_vel > self.max_vel else current_vel
            else:
                break

            self.position[1] += current_vel
            #if self.funtionality["set_pos_viz"] is not None:

            tmr = time.time()
            while time.time() - tmr < abs(current_vel*2):
                if self.funtionality["set_pos_viz"] is not None:
                    self.funtionality["set_pos_viz"](self.position)
                sleep(.001)
        return {"Position3D": self.position}

    def PlaceBlock(self, params: dict):

        return params

    def GetPosition(self, params: dict):
        formatPrint(self, f"Get Position {params}")
        pos = copy(self.position)
        pos[2] += .05
        pos[1] += .2
        pos[0] += .1
        return {"Position3D": pos}

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
