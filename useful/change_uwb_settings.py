#!/usr/bin/env python
"""change_uwb_settings.py - Changes the UWB settings of all devices listed.

This assumes all listed devices are on the same UWB settings already,
otherwise you should run the set_same_settings.py script, as that one
finds all devices on all settings.
"""

from pypozyx import *
from pypozyx.definitions.registers import POZYX_UWB_CHANNEL, POZYX_UWB_RATES, POZYX_UWB_PLEN

class ChangeUWBSettings:

    def __init__(self, pozyx, uwb_settings, devices=[], set_local=True, save_to_flash=True):
        self.pozyx = pozyx
        self.uwb_settings = uwb_settings
        self.devices = devices
        self.set_local = set_local
        self.save_to_flash = save_to_flash
        self.get_start_settings()

    def get_start_settings(self):
        self.start_settings = UWBSettings()
        status = self.pozyx.getUWBSettings(self.start_settings)
        if status == POZYX_SUCCESS:
            print("current UWB settings: %s" % self.start_settings)
        else:
            print("current UWB settings could not be retrieved, terminating")
            raise Exception
        return status

    def run(self):
        for tag in self.devices:
            self.set_to_settings(tag)
        if not self.set_local:
            self.pozyx.setUWBSettings(self.start_settings)
        else:
            if save_to_flash:
                self.pozyx.saveUWBSettings()

    def set_to_settings(self, remote_id):
        self.pozyx.setUWBSettings(self.start_settings)
        self.pozyx.setUWBSettings(self.uwb_settings, remote_id)
        self.pozyx.setUWBSettings(self.uwb_settings)
        whoami = SingleRegister()
        status = self.pozyx.getWhoAmI(whoami, remote_id)
        if whoami[0] != 0x67 or status != POZYX_SUCCESS:
            print("Changing UWB settings on device 0x%0.4x failed" % remote_id)
            return
        else:
            print("Settings successfully changed on device 0x%0.4x" % remote_id)
        if self.save_to_flash:
            status = self.pozyx.saveUWBSettings(remote_id)
            if status != POZYX_SUCCESS:
                print("\tAnd saving settings failed.")
            else:
                print("\tAnd saving settings succeeded")

if __name__ == '__main__':
    # new uwb_settings
    uwb_settings = UWBSettings(channel=2,
                               bitrate=2,
                               prf=2,
                               plen=0x04,
                               gain_db=15.0)

    # set to True if local tag needs to change settings as well.
    set_local = True

    # set to True if needed to save to flash
    save_to_flash = True

    # list of IDs to set UWB settings for. example devices = [0x6001, 0x6002,
    # 0x6799]
    devices = []

    # pozyx
    pozyx = PozyxSerial(get_serial_ports()[0].device)

    # initialize the class
    c = ChangeUWBSettings(pozyx, uwb_settings, devices, set_local, save_to_flash)

    # run the functionality
    c.run()
