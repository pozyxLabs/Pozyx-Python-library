#!/usr/bin/env python
"""
Pozyx basic troubleshooting (c) Pozyx Labs 2017

If you're experiencing trouble with Pozyx, this should be your first step to check for problems.
Please read the article on https://www.pozyx.io/Documentation/Tutorials/troubleshoot_basics/Python
"""

from pypozyx import *
from pypozyx.definitions.registers import POZYX_WHO_AM_I

def get_device_type(hardware_version):
    if hardware_version >> 5 == 1:
        return 'tag'
    else:
        return 'anchor'

def device_check(pozyx, remote_id=None):
    data = Data([0] * 5)

    if remote_id is None:
        print("local device")
    else:
        print("device 0x%0.4x" % remote_id)

    pozyx.getRead(POZYX_WHO_AM_I, data, remote_id=remote_id)
    print("who am i: 0x%0.2x" % data[0])
    print("firmware version: %i.%i" % (data[1] >> 4, data[1] % 0x10))
    print("hardware: %s v1.%i" % (get_device_type(data[2]), data[2] % 0x20))
    print("self test result: %s" % bin(data[3]))
    print("error: 0x%0.2x" % data[4])
    print("error message: %s" % pozyx.getErrorMessage(data[4]))


def network_check_discovery(pozyx, remote_id=None):
    pozyx.clearDevices(remote_id)
    if pozyx.doDiscovery(discovery_type=POZYX_DISCOVERY_ALL_DEVICES, remote_id=remote_id) == POZYX_SUCCESS:
        print("Found devices:")
        pozyx.printDeviceList(remote_id, include_coordinates=False)


if __name__ == '__main__':
    serial_port = get_first_pozyx_serial_port()
    if serial_port is None:
        print("No Pozyx connected. Check your USB cable or your driver!")
        quit()

    pozyx = PozyxSerial(serial_port)

    # change to remote ID for troubleshooting that device
    remote_id = None

    device_check(pozyx, remote_id)
    network_check_discovery(pozyx, remote_id)
