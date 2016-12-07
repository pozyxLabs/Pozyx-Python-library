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

remote_id = 0x685F           # the network ID of the remote device
remote = False               # whether to use the given remote device for ranging
if not remote:
    remote_id = None

destination_id = 0x6830      # network ID of the ranging destination
range_step_mm = 1000         # distance that separates the amount of LEDs lighting up.


# device_range = DeviceRange()
# status = p.doRanging(destination_id, device_range, remote_id)
status=1
if status:
    # print device_range.data
    cira=np.array([])

    rangeoffset = range(600,1015,49)
    for k in rangeoffset:
        # 96 = output bytes length
        Buff = Buffer([0]*96,size=2,signed=1)
        # 49 = data length
        p.getDeviceCirData(k,49,Buff)
        import ipdb
        ipdb.set_trace()
        cont = np.array(Buff.data)
        cir = cont[0::2] + 1j*cont[1::2]
        cira = np.append(cira,cir)
    plt.ion()
    plt.plot(20*np.log10(abs(cira[:-36])))
    plt.show()
else:
    print 'fail'

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
