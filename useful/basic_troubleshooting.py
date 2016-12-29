#!/usr/bin/env python
"""
Pozyx basic troubleshooting (c) Pozyx Labs 2016

If you're experiencing trouble with Pozyx, this should be your first step to check for problems.
Please read the article on https://www.pozyx.io/Documentation/Tutorials/troubleshoot_basics/Python
"""

from pypozyx import *
from pypozyx.definitions.registers import POZYX_WHO_AM_I


def device_check(pozyx, remote_id=None):

    data = Data([0] * 5)

    if remote_id is None:
        print("local device")
    else:
        print("device 0x%0.4x" % remote_id)

    pozyx.getRead(POZYX_WHO_AM_I, data, remote_id=remote_id)
    print('who am i: 0x%0.2x' % data[0])
    print('firmware version: 0x%0.2x' % data[1])
    print('hardware version: 0x%0.2x' % data[2])
    print('self test result: %s' % bin(data[3]))
    print('error: 0x%0.2x' % data[4])


def network_check_discovery(pozyx, remote_id=None):
    pozyx.clearDevices(remote_id)
    if pozyx.doDiscovery(discovery_type=POZYX_DISCOVERY_ALL_DEVICES, remote_id=remote_id) == POZYX_SUCCESS:
        pozyx.printDeviceList(remote_id)


if __name__ == '__main__':
    port = get_serial_ports()[0].device
    pozyx = PozyxSerial(port)
    # use this for local devices
    device_check(pozyx)
    # uncomment this for remote devices
    # device_check(pozyx, 0x6000)

    # use this for checking devices in range
    network_check_discovery(pozyx)
    # uncomment this for checking devices in range for remote devices.
    # network_check_discovery(pozyx, 0x6000)
