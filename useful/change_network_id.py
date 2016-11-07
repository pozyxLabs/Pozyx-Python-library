#!/usr/bin/env python
"""change_network_id.py - Changes a local/remote device's network ID."""
from pypozyx import *
from pypozyx.definitions.registers import *
# remote_id = 0x6000

remote = False
remote_id = None
if remote:
    remote_id = 0x6F5E

port = '/dev/ttyACM3'

new_id = 0x6000


class SetNetworkID():

    def __init__(self, port, new_id, remote_id=None, save_to_flash=False):
        try:
            self.pozyx = PozyxSerial(port)
        except:
            print('ERROR: Unable to connect to Pozyx, wrong port')
            raise SystemExit

        registers = [POZYX_NETWORK_ID]
        if self.pozyx.setNetworkId(new_id, remote_id) == POZYX_FAILURE:
            print("Setting Network ID failed")
            return
        if save_to_flash:
            if remote:
                remote_id = new_id
            if self.pozyx.saveConfiguration(POZYX_FLASH_REGS, registers, remote_id) == POZYX_FAILURE:
                print("Saving the Network ID failed")
                return
        print("Everything worked!")


if __name__ == "__main__":
    SetNetworkID(port, new_id, remote_id, True)
