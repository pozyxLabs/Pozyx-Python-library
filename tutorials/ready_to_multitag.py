#!/usr/bin/env python
"""
multitag_positioning.py - Tutorial intended to show how multitag positioning works.

If you are unfamiliar with the Pozyx platform but want to start on this right away,
I would recommend you to check out the ready to localize tutorial first before
coming to this one.
"""
from pypozyx import *


class ReadyToMultitag():
    """Continuously performs multitag positioning"""

    def __init__(self, pozyx, tags, anchors, algorithm=POZYX_POS_ALG_UWB_ONLY, dimension=POZYX_3D, height=1000, remote_id=None):
        self.pozyx = pozyx
        self.tags = tags
        self.anchors = anchors
        self.algorithm = algorithm
        self.dimension = dimension
        self.height = height
        self.remote_id = remote_id

    def setup(self):
        """Sets up the Pozyx for positioning by calibrating its anchor list."""
        print("------------POZYX MULTITAG POSITIONING V1.0 - -----------\nNOTES: \n- Parameters required:\n\t- Anchors for calibration\n\t- Tags to work with\n\n- System will manually calibration\n\n- System will auto start positioning\n- -----------POZYX MULTITAG POSITIONING V1.0 ------------\nSTART Positioning: ")
        self.setAnchorsManual()

    def loop(self):
        """Performs positioning and prints the results."""
        for tag in self.tags:
            position = Coordinates()
            status = self.pozyx.doPositioning(
                position, self.dimension, self.height, self.algorithm, remote_id=tag)
            if status == POZYX_SUCCESS:
                self.printCoordinates(tag, position)
            else:
                self.printErrorCode(tag)

    def setAnchorsManual(self):
        """Adds the manually measured anchors to the Pozyx's device list one for one."""
        for tag in self.tags:
            status = self.pozyx.clearDevices(tag)
            for anchor in self.anchors:
                status &= self.pozyx.addDevice(anchor, tag)
            self.printConfigurationResult(status, tag)

    def printConfigurationResult(self, status, tag_id):
        """Prints the configuration result"""
        if tag_id is None:
            tag_id = "LOCAL"
        else:
            tag_id = "0x%0.4x" % tag_id
        if status == POZYX_SUCCESS:
            print("Configuration of tag %s: success" % tag_id)
        else:
            print("Configuration of tag %s: failed" % tag_id)

    def printCoordinates(self, tag_id, pos):
        """Prints the coordinates in a human-readable way."""
        if tag_id is None:
            print("ID: LOCAL, x(mm): %i, y(mm): %i, z(mm): %i" %
                  (pos.x, pos.y, pos.z))
        else:
            print("ID: 0x%0.4x, x(mm): %i, y(mm): %i, z(mm): %i" %
                  (tag_id, pos.x, pos.y, pos.z))

    def printErrorCode(self, tag_id):
        """Prints both failure and resulting error code"""
        error_code = SingleRegister()
        status = self.pozyx.getErrorCode(error_code, self.remote_id)
        if tag_id is None:
            tag_id = "LOCAL"
        else:
            tag_id = "0x%0.4x" % tag_id
        if status == POZYX_SUCCESS:
            print("Error positioning of tag %s, code: %s" % (tag_id, str(error_code)))
        else:
            print("Error positioning of tag %s, error couldn't be read." % tag_id)


if __name__ == "__main__":
    # shortcut to not have to find out the port yourself
    port = get_serial_ports()[0].device
    remote_id = 0x1000                     # remote device network ID
    remote = False                         # whether to use a remote device
    if not remote:
        remote_id = None

    tags = [0x0001, 0x0002, 0x0003]        # remote tags
    # necessary data for calibration
    anchors = [DeviceCoordinates(0x0001, 1, Coordinates(0, 0, 2000)),
               DeviceCoordinates(0x0002, 1, Coordinates(3000, 0, 2000)),
               DeviceCoordinates(0x0003, 1, Coordinates(0, 3000, 2000)),
               DeviceCoordinates(0x0004, 1, Coordinates(3000, 3000, 2000))]

    algorithm = POZYX_POS_ALG_UWB_ONLY     # positioning algorithm to use
    dimension = POZYX_3D                   # positioning dimension
    height = 1000                          # height of device, required in 2.5D positioning

    pozyx = PozyxSerial(port)
    r = ReadyToMultitag(pozyx, tags, anchors, algorithm, dimension, height, remote_id)
    r.setup()
    while True:
        r.loop()
