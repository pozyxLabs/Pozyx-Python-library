#!/usr/bin/env python
"""change_uwb_settings.py - Changes the UWB settings of all devices listed.

It can also print the currently saved device list of the Pozyx

This assumes all listed devices are on the same UWB settings already,
otherwise you should run the set_same_settings.py script, as that one
finds all devices on all settings.
"""

from pypozyx import *


class MultiDeviceListCongifuration(object):

    def __init__(self, pozyx, tags=[None]):
        self.pozyx = pozyx
        self.tags = tags

    def configure_anchor_list(self, anchors):
        """Configures the tags with the anchors"""
        for tag in self.tags:
            pozyx.configureAnchors(anchors, remote_id=tag)
            pozyx.saveNetwork(remote_id=tag)

    def read_anchor_list(self):
        """Reads the anchor list the device is configured with"""
        for tag in self.tags:
            if tag is None:
                print("LOCAL TAG")
            else:
                print("TAG ID: 0x%0.4x" % tag)
            self.pozyx.printDeviceList(tag)


if __name__ == '__main__':
    # virtual serial port of the Pozyx
    port = get_serial_ports()[0].device

    # To configure the devicelist anew, set to True
    # To read the currently configured device list, set to False
    configure = True

    # list of IDs to configure with the anchors.
    # add None if you want to configure the local device as well.
    tags = [None]

    # list of anchors with respective coordinates for the configuration
    anchors = [DeviceCoordinates(0x0001, 1, Coordinates(0, 0, 0)),
               DeviceCoordinates(0x0002, 1, Coordinates(1000, 0, 0)),
               DeviceCoordinates(0x0003, 1, Coordinates(0, 1000, 0)),
               DeviceCoordinates(0x0004, 1, Coordinates(1000, 1000, 0))]

    pozyx = PozyxSerial(port)

    d = MultiDeviceListConfiguration(pozyx, tags)

    if configure:
        d.configure_anchor_list(anchors)
    else:
        d.read_anchor_list(pozyx)
