#!/usr/bin/env python
"""
ready_to_localize.py - Tutorial intended to show how to perform positioning with Pozyx.

It is planned to make a tutorial on the Pozyx site as well just like there is now
one for the Arduino, but their operation is very similar.
You can find the Arduino tutorial here:
    https://www.pozyx.io/Documentation/Tutorials/ready_to_localize
"""
from pypozyx import *


class ReadyToLocalize():
    """Continuously calls the Pozyx positioning function and prints its position."""

    def __init__(self, pozyx, anchors, algorithm=POZYX_POS_ALG_UWB_ONLY, dimension=POZYX_3D, height=1000, remote_id=None):
        self.pozyx = pozyx
        self.anchors = anchors
        self.algorithm = algorithm
        self.dimension = dimension
        self.height = height
        self.remote_id = remote_id

    def setup(self):
        """Sets up the Pozyx for positioning by calibrating its anchor list."""
        print("------------POZYX POSITIONING V1.0 - -----------\nNOTES: \n- No parameters required.\n\n- System will auto start calibration\n\n- System will auto start positioning\n- -----------POZYX POSITIONING V1.0 ------------\nSTART Ranging: ")
        self.pozyx.clearDevices(self.remote_id)
        self.setAnchorsManual()
        self.printConfigurationResult()

    def loop(self):
        """Performs positioning and prints the results."""
        position = Coordinates()
        status = self.pozyx.doPositioning(
            position, self.dimension, self.height, self.algorithm, remote_id=self.remote_id)
        if status == POZYX_SUCCESS:
            self.printCoordinates(position)
        else:
            self.printErrorCode()

    def printErrorCode(self):
        error_code = SingleRegister()
        status = self.pozyx.getErrorCode(error_code, self.remote_id)
        if status == POZYX_SUCCESS:
            print("Error positioning, code: %s" % str(error_code))
        else:
            print("Error positioning, error couldn't be read.")

    def printCoordinates(self, pos):
        """Prints the coordinates in a human-readable way."""
        print("x(mm): {pos.x}, y(mm): {pos.y}, z(mm): {pos.z}".format(pos=pos))

    def setAnchorsManual(self):
        """Adds the manually measured anchors to the Pozyx's device list one for one."""
        status = self.pozyx.clearDevices(self.remote_id)
        for anchor in self.anchors:
            status &= self.pozyx.addDevice(anchor, self.remote_id)
        return status

    def printConfigurationResult(self):
        """Prints the anchor configuration results in a human-readable way."""
        list_size = SingleRegister()

        status = self.pozyx.getDeviceListSize(list_size, self.remote_id)
        print("List size: {0}".format(list_size[0]))
        if list_size[0] == 0:
            print("Calibration failed.\n%s" % self.pozyx.getSystemError())
            return
        device_list = DeviceList(list_size=list_size[0])
        status = self.pozyx.getDeviceIds(device_list, self.remote_id)
        print("Calibration result:")
        print("Anchors found: {0}".format(list_size[0]))
        print("Anchor IDs: ", device_list)

        for i in range(list_size[0]):
            anchor_coordinates = Coordinates()
            status = self.pozyx.getDeviceCoordinates(
                device_list[i], anchor_coordinates, self.remote_id)
            print("ANCHOR,0x%0.4x,%s" % (device_list[i], str(anchor_coordinates)))

if __name__ == "__main__":
    # shortcut to not have to find out the port yourself
    port = get_serial_ports()[0].device
    remote_id = 0x1000                     # remote device network ID
    remote = False                         # whether to use a remote device
    if not remote:
        remote_id = None
    # necessary data for calibration
    anchors = [DeviceCoordinates(0x0001, 1, Coordinates(0, 0, 2000)),
               DeviceCoordinates(0x0002, 1, Coordinates(3000, 0, 2000)),
               DeviceCoordinates(0x0003, 1, Coordinates(0, 3000, 2000)),
               DeviceCoordinates(0x0004, 1, Coordinates(3000, 3000, 2000))]

    algorithm = POZYX_POS_ALG_UWB_ONLY     # positioning algorithm to use
    dimension = POZYX_3D                   # positioning dimension
    height = 1000                          # height of device, required in 2.5D positioning

    pozyx = PozyxSerial(port)
    r = ReadyToLocalize(pozyx, anchors, algorithm, dimension, remote_id)
    r.setup()
    while True:
        r.loop()
