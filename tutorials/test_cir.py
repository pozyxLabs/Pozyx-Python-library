#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
ready_to_range.py - Tutorial intended to show how to perform ranging between two Pozyx devices.

It is planned to make a tutorial on the Pozyx site as well just like there is now
one for the Arduino, but their operation is very similar.
You can find the Arduino tutorial here:
    https://www.pozyx.io/Documentation/Tutorials/ready_to_range
"""
from pypozyx import *

from pypozyx.definitions.registers import *

import matplotlib.pyplot as plt
import numpy as np


for k in range(5):
    port = '/dev/ttyACM'+str(k)                # COM port of the Pozyx device
    print port
    try:
        p = PozyxSerial(port)
        break
    except:
        pass

remote_id = 0x605D           # the network ID of the remote device
remote = False               # whether to use the given remote device for ranging
if not remote:
    remote_id = None

destination_id = 0x6830      # network ID of the ranging destination
range_step_mm = 1000         # distance that separates the amount of LEDs lighting up.



# r=DeviceRange()
# p.useFunction(POZYX_DEVICE_GETRANGEINFO,Data([0x6830],'H'),r,None)
# p.useFunction(POZYX_DO_RANGING,Data([0x6830],'H'),r,None)
device_range = DeviceRange()

status = p.doRanging(destination_id, device_range, remote_id)

cira=np.array([])
rangeoffset = range(0,1015,49)
for k in rangeoffset:
    offset = hex(k)
    # 0x31 = 49 = data length
    param = Data([eval(offset),0x31],'Hb')
    data_format = ['h']*96
    data_format = ''.join(data_format)
    container = Data([0]*96,data_format)
    p.regFunction(POZYX_CIR_DATA, param, container)
    cont = np.array(container.data)
    cir = cont[0::2] + 1j*cont[1::2]
    cira = np.append(cira,cir)

plt.plot(20*np.log10(abs(cira)))
plt.show()

# lcir.extend(cir)

# plt.plot(lcir)
# plt.show()

# r=DeviceRange()
# #p.useFunction(POZYX_DEVICE_GETRANGEINFO,Data([0x6830],'H'),r,None)
# p.regFunction(POZYX_DEVICE_GETRANGEINFO,Data([0x6830],'H'),r)

# class cirparam(ByteStructure):
#     byte_size = 12
#     data_format = 'III'

#     def __init__(self, timestamp=0, distance=0, RSS=0):
