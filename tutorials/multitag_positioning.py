#!/usr/bin/env python
"""
multitag_positioning.py - Tutorial intended to show how multitag positioning works.

If you are unfamiliar with the Pozyx platform but want to start on this right away,
I would recommend you to check out the ready to localize tutorial first before
coming to this one.
"""
from time import sleep

from pypozyx import *

port = '/dev/ttyACM0'

# include None if you want to perform positioning locally as well.
tags = [0x0001, 0x0002, 0x0003]
# necessary data for calibration
num_anchors = 4
anchor_ids = [0x605D, 0x2000, 0x6044, 0x6639]
heights = [1920, 2025, 1330, 1510]
# manual calibration
anchors_x = [0, -10, 3999, 4060]
anchors_y = [0, 3950, 0, 3945]
# for 2.5D
height = 1000


class MultitagPositioning():
    """Continuously performs multitag positioning"""

    def __init__(self, port):
        try:
            self.pozyx = PozyxSerial(port)
        except:
            print('ERROR: Unable to connect to Pozyx, wrong port')
            raise SystemExit

        self.setup()
        while True:
            self.loop()

    def setup(self):
        """Sets up the Pozyx for positioning by calibrating its anchor list."""
        print("------------POZYX MULTITAG POSITIONING V1.0 - -----------\nNOTES: \n- Parameters required:\n\t- Anchors for calibration\n\t- Tags to work with\n\n- System will manually calibration\n\n- System will auto start positioning\n- -----------POZYX MULTITAG POSITIONING V1.0 ------------\nSTART Positioning: ")
        # Adds
        self.pozyx.clearDevices()
        self.setAnchorsManual()

    def loop(self):
        """Performs positioning and prints the results."""
        for tag in tags:
            position = Coordinates()
            status = self.pozyx.doPositioning(
                position, remote_id=tag)
            if status == POZYX_SUCCESS:
                self.printCoordinates(tag, position)

    def printCoordinates(self, tag_id, pos):
        """Prints the coordinates in a human-readable way."""
        print("ID: 0x%0.4x, x(mm): %i, y(mm): %i, z(mm): %i" %
              (tag_id, pos.x, pos.y, pos.z))

    def setAnchorsManual(self):
        """Adds the manually measured anchors to the Pozyx's device list one for one."""
        for i in range(num_anchors):
            anchor_coordinates = Coordinates(
                anchors_x[i], anchors_y[i], heights[i])
            anchor = DeviceCoordinates(anchor_ids[i], 0x1, anchor_coordinates)
            for tag in tags:
                status = self.pozyx.addDevice(anchor, tag)
                if status != POZYX_SUCCESS:
                    print("Calibration of tag with 0x%0.4x failed" % tag)
                else:
                    print("Calibration of tag with 0x%0.4x success" % tag)

if __name__ == "__main__":
    m = MultitagPositioning(port)
