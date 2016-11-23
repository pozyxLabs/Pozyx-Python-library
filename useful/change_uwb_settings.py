#!/usr/bin/env python
"""change_uwb_settings.py - Changes the UWB settings of all devices listed.

It can also print the currently saved device list of the Pozyx

This assumes all listed devices are on the same UWB settings already,
otherwise you should run the set_same_settings.py script, as that one
finds all devices on all settings.
"""

from pypozyx import *
from pypozyx.definitions.registers import POZYX_UWB_CHANNEL, POZYX_UWB_RATES, POZYX_UWB_PLEN

# virtual serial port of the Pozyx
port = 'COM12'

# set to True if local tag needs to change settings as well.
set_local = True

# list of IDs to set UWB settings for
tags = [0x0001]

# list of anchors with respective coordinates for the configuration
uwb_settings = UWBSettings(channel=2,
                           bitrate=2,
                           prf=2,
                           plen=0x04,
                           gain_db=15.0)

def set_uwb_settings(pozyx):
    """Configures the tags with the anchors"""
    regs = [POZYX_UWB_CHANNEL, POZYX_UWB_RATES, POZYX_UWB_PLEN]
    for tag in tags:
        pozyx.setUWBSettings(uwb_settings, tag)
        pozyx.saveRegisters(regs, tag)
    if set_local:
        pozyx.setUWBSettings(uwb_settings)
        pozyx.saveRegisters(regs)

if __name__ == '__main__':
    pozyx = PozyxSerial(port)
    set_uwb_settings(pozyx)
