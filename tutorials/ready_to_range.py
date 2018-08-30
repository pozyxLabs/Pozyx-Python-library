#!/usr/bin/env python
"""
The Pozyx ready to range tutorial (c) Pozyx Labs
Please read the tutorial that accompanies this sketch: https://www.pozyx.io/Documentation/Tutorials/ready_to_range/Python

This demo requires two Pozyx devices. It demonstrates the ranging capabilities and the functionality to
to remotely control a Pozyx device. Move around with the other Pozyx device.

This demo measures the range between the two devices. The closer the devices are to each other, the more LEDs will
light up on both devices.
"""
from pypozyx import (PozyxSerial, PozyxConstants, version,
                     SingleRegister, DeviceRange, POZYX_SUCCESS, POZYX_FAILURE, get_first_pozyx_serial_port)

from pypozyx.tools.version_check import perform_latest_version_check


class ReadyToRange(object):
    """Continuously performs ranging between the Pozyx and a destination and sets their LEDs"""

    def __init__(self, pozyx, destination_id, range_step_mm=1000, protocol=PozyxConstants.RANGE_PROTOCOL_PRECISION,
                 remote_id=None):
        self.pozyx = pozyx
        self.destination_id = destination_id
        self.range_step_mm = range_step_mm
        self.remote_id = remote_id
        self.protocol = protocol

    def setup(self):
        """Sets up both the ranging and destination Pozyx's LED configuration"""
        print("------------POZYX RANGING V{} -------------".format(version))
        print("NOTES: ")
        print(" - Change the parameters: ")
        print("\tdestination_id(target device)")
        print("\trange_step(mm)")
        print("")
        print("- Approach target device to see range and")
        print("led control")
        print("")
        if self.remote_id is None:
            for device_id in [self.remote_id, self.destination_id]:
                self.pozyx.printDeviceInfo(device_id)
        else:
            for device_id in [None, self.remote_id, self.destination_id]:
                self.pozyx.printDeviceInfo(device_id)
        print("")
        print("- -----------POZYX RANGING V{} -------------".format(version))
        print("")
        print("START Ranging: ")

        # make sure the local/remote pozyx system has no control over the LEDs.
        led_config = 0x0
        self.pozyx.setLedConfig(led_config, self.remote_id)
        # do the same for the destination.
        self.pozyx.setLedConfig(led_config, self.destination_id)
        # set the ranging protocol
        self.pozyx.setRangingProtocol(self.protocol, self.remote_id)

    def loop(self):
        """Performs ranging and sets the LEDs accordingly"""
        device_range = DeviceRange()
        status = self.pozyx.doRanging(
            self.destination_id, device_range, self.remote_id)
        if status == POZYX_SUCCESS:
            print(device_range)
            if self.ledControl(device_range.distance) == POZYX_FAILURE:
                print("ERROR: setting (remote) leds")
        else:
            error_code = SingleRegister()
            status = self.pozyx.getErrorCode(error_code)
            if status == POZYX_SUCCESS:
                print("ERROR Ranging, local %s" %
                      self.pozyx.getErrorMessage(error_code))
            else:
                print("ERROR Ranging, couldn't retrieve local error")

    def ledControl(self, distance):
        """Sets LEDs according to the distance between two devices"""
        status = POZYX_SUCCESS
        ids = [self.remote_id, self.destination_id]
        # set the leds of both local/remote and destination pozyx device
        for id in ids:
            status &= self.pozyx.setLed(4, (distance < range_step_mm), id)
            status &= self.pozyx.setLed(3, (distance < 2 * range_step_mm), id)
            status &= self.pozyx.setLed(2, (distance < 3 * range_step_mm), id)
            status &= self.pozyx.setLed(1, (distance < 4 * range_step_mm), id)
        return status


if __name__ == "__main__":
    # Check for the latest PyPozyx version. Skip if this takes too long or is not needed by setting to False.
    check_pypozyx_version = True
    if check_pypozyx_version:
        perform_latest_version_check()

    # hardcoded way to assign a serial port of the Pozyx
    serial_port = 'COM12'

    # the easier way
    serial_port = get_first_pozyx_serial_port()
    if serial_port is None:
        print("No Pozyx connected. Check your USB cable or your driver!")
        quit()

    remote_id = 0x605D           # the network ID of the remote device
    remote = False               # whether to use the given remote device for ranging
    if not remote:
        remote_id = None

    destination_id = 0x6e66      # network ID of the ranging destination
    # distance that separates the amount of LEDs lighting up.
    range_step_mm = 1000

    # the ranging protocol, other one is PozyxConstants.RANGE_PROTOCOL_PRECISION
    ranging_protocol = PozyxConstants.RANGE_PROTOCOL_PRECISION

    pozyx = PozyxSerial(serial_port)
    r = ReadyToRange(pozyx, destination_id, range_step_mm,
                     ranging_protocol, remote_id)
    r.setup()
    while True:
        r.loop()
