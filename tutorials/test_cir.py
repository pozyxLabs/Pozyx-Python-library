#!/usr/bin/env python
"""
ready_to_range.py - Tutorial intended to show how to perform ranging between two Pozyx devices.

It is planned to make a tutorial on the Pozyx site as well just like there is now
one for the Arduino, but their operation is very similar.
You can find the Arduino tutorial here:
    https://www.pozyx.io/Documentation/Tutorials/ready_to_range
"""
from pypozyx import *

from pypozyx.definitions.registers import *



port = '/dev/ttyACM1'                # COM port of the Pozyx device

remote_id = 0x605D           # the network ID of the remote device
remote = False               # whether to use the given remote device for ranging
if not remote:
    remote_id = None

destination_id = 0x6830      # network ID of the ranging destination
range_step_mm = 1000         # distance that separates the amount of LEDs lighting up.

p = PozyxSerial(port)




param = Data([0x0000,49],'Hi')



data_format = ['i']*98
data_format = ''.join(data_format)
cir = Data([],data_format)


p.useFunction(POZYX_CIR_DATA, params, cir, remote_id)