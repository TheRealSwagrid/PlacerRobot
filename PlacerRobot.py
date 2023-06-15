#!/usr/bin/env python
import signal
import sys

from AbstractVirtualCapability import AbstractVirtualCapability, VirtualCapabilityServer, formatPrint


class PlacerRobot(AbstractVirtualCapability):
    def __init__(self, server):
        super().__init__(server)
        self.position = [0., 0., 0.]
        self.funtionality = {"set_pos_viz": None}
        self.max_vel = 0.1
        self.acc = 0.005

    def MoveBy(self, params: dict):
        formatPrint(self, f"Forwarding with {params}")
        val = params["SimpleDoubleParameter"]

        goal = self.position
        goal[1] += val

        current_vel = 0
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
            if self.funtionality["set_pos_viz"] is not None:
                self.funtionality["set_pos_viz"](self.position)

        return {"Position3D":self.position}

    def GetPosition(self, params: dict):
        formatPrint(self, f"Get Position {params}")
        return {"Position3D": self.position}

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
