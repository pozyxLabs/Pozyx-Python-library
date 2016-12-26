#!/usr/bin/env python
"""change_network_id.py - Changes a local/remote device's network ID."""

from pypozyx import *
from pypozyx.definitions.registers import *


def set_new_id(pozyx, new_id, remote_id):
    print("Setting the Pozyx ID to 0x%0.4x" % new_id)
    pozyx.setNetworkId(new_id, remote_id)
    if pozyx.saveConfiguration(POZYX_FLASH_REGS, [POZYX_NETWORK_ID], remote_id) == POZYX_SUCCESS:
        print("Saving new ID successful! Resetting system...")
        if pozyx.resetSystem(remote_id) == POZYX_SUCCESS:
            print("Done")


if __name__ == "__main__":

    port = get_serial_ports()[0].device

    remote = False
    remote_id = 0x6F5E
    if not remote:
        remote_id = None

    new_id = 0xA004

    pozyx = PozyxSerial(port)
    set_new_id(port, pozyx, new_id, remote_id, True)
