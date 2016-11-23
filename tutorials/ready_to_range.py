#!/usr/bin/env python
"""
ready_to_range.py - Tutorial intended to show how to perform ranging between two Pozyx devices.

It is planned to make a tutorial on the Pozyx site as well just like there is now
one for the Arduino, but their operation is very similar.
You can find the Arduino tutorial here:
    https://www.pozyx.io/Documentation/Tutorials/ready_to_range
"""
from pypozyx import *


class ReadyToRange():
    """Continuously performs ranging between the Pozyx and a destination and sets their LEDs"""

    def __init__(self, pozyx, destination_id, range_step_mm=1000, remote_id=None):
        self.pozyx = pozyx
        self.destination_id = destination_id
        self.range_step_mm = range_step_mm
        self.remote_id = remote_id

    def setup(self):
        """Sets up both the ranging and destination Pozyx's LED configuration"""
        print("------------POZYX RANGING V1.0 - -----------\nNOTES: \n - Change the parameters: \n\tdestination_id(target device)\n\trange_step(mm)\n- Approach target device to see range and\nled control\n- -----------POZYX RANGING V1.0 ------------\nSTART Ranging: ")
        led_config = 0x0
        # make sure the local/remote pozyx system has no control over the LEDs.
        self.pozyx.setLedConfig(led_config, self.remote_id)
        # do the same for the destination.
        self.pozyx.setLedConfig(led_config, self.destination_id)

    def loop(self):
        """Performs ranging and sets the LEDs accordingly"""
        device_range = DeviceRange()
        status = self.pozyx.doRanging(self.destination_id, device_range, self.remote_id)
        if status == POZYX_SUCCESS:
            print(device_range)
            self.ledControl(device_range.distance)
        else:
            print("ERROR: ranging")

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
    port = 'COM1'                # COM port of the Pozyx device

    remote_id = 0x605D           # the network ID of the remote device
    remote = False               # whether to use the given remote device for ranging
    if not remote:
        remote_id = None

    destination_id = 0x1000      # network ID of the ranging destination
    range_step_mm = 1000         # distance that separates the amount of LEDs lighting up.

    pozyx = PozyxSerial(port)
    r = ReadyToRange(pozyx, destination_id, range_step_mm, remote_id)
    r.setup()
    while True:
        r.loop()
