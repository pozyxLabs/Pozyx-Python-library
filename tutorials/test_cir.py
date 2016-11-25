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



port = '/dev/ttyACM4'                # COM port of the Pozyx device

remote_id = 0x605D           # the network ID of the remote device
remote = False               # whether to use the given remote device for ranging
if not remote:
    remote_id = None

destination_id = 0x6830      # network ID of the ranging destination
range_step_mm = 1000         # distance that separates the amount of LEDs lighting up.

p = PozyxSerial(port)




param = Data([0x0000,0x0000,49],'HHI')



data_format = ['i']*99
data_format = ''.join(data_format)
cir = Data([0]*99,data_format)

print p.regFunction(POZYX_CIR_DATA, param, cir)


r=DeviceRange()
#p.useFunction(POZYX_DEVICE_GETRANGEINFO,Data([0x6830],'H'),r,None)
p.regFunction(POZYX_DEVICE_GETRANGEINFO,Data([0x6830],'H'),r)

# class cirparam(ByteStructure):
#     byte_size = 12
#     data_format = 'III'

#     def __init__(self, timestamp=0, distance=0, RSS=0):
