#!/usr/bin/env python
"""
multitag_positioning.py - Tutorial intended to show how multitag positioning works.

If you are unfamiliar with the Pozyx platform but want to start on this right away,
I would recommend you to check out the ready to localize tutorial first before
coming to this one.

Why is there no automatic calibration? Because this isn't supported remotely.
"""
from time import sleep
from pythonosc import osc_message_builder, udp_client

from pypozyx import *

port = '/dev/cu.usbmodem1411'

# include None if you want to perform positioning locally as well.
tags = [0x606B, 0x6062, 0x607A, 0x6078, 0x6061, 0x607F, 0x607E]
# necessary data for calibration
num_anchors = 4
anchor_ids = [0x605D, 0x2000, 0x6044, 0x6639]
heights = [1000, 1000, 1000, 1000]
# manual calibration
anchors_x = [0, 3657, 0, 3657]
anchors_y = [0, 0, 2794, 2717]
# for 2.5D
height = 1000

# for networking
ip = "192.168.0.10"  # localhost
network_port = 3333



class MultitagPositioningUDP():
    """Continuously performs multitag positioning"""

    def __init__(self, port):
        try:
            self.pozyx = PozyxSerial(port)
        except:
            print('ERROR: Unable to connect to Pozyx, wrong port')
            raise SystemExit

        self.client = udp_client.SimpleUDPClient(ip, network_port)

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
                self.publishPositionOSC(tag, position)

    def publishPositionOSC(self, tag_id, pos):
        """Publishes the ID and position over the socket as bytes."""
        msg_builder = osc_message_builder.OscMessageBuilder("/tag0x%0.4x" % tag_id)
        msg_builder.add_arg(int(pos.x), "i")
        msg_builder.add_arg(int(pos.y), "i")
        self.client.send(msg_builder.build())

    def setAnchorsManual(self):
        """Adds the manually measured anchors to the Pozyx's device list one for one."""
        for tag in tags:
            status = POZYX_SUCCESS
            for i in range(num_anchors):
                anchor_coordinates = Coordinates(
                    anchors_x[i], anchors_y[i], heights[i])
                anchor = DeviceCoordinates(anchor_ids[i], 0x1, anchor_coordinates)

                status &= self.pozyx.addDevice(anchor, tag)
            if status != POZYX_SUCCESS:
                print("Calibration of tag with 0x%0.4x failed" % tag)
            else:
                print("Calibration of tag with 0x%0.4x success" % tag)

if __name__ == "__main__":
    m = MultitagPositioningUDP(port)
