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


port = '/dev/ttyACM1'
p = PozyxSerial(port)

remote_id = 0x685F           # the network ID of the remote device
remote = False               # whether to use the given remote device for ranging
if not remote:
    remote_id = None

destination_id = 0x6830      # network ID of the ranging destination
range_step_mm = 1000         # distance that separates the amount of LEDs lighting up.


device_range = DeviceRange()
status = p.doRanging(destination_id, device_range, remote_id)
if status:
    cira = np.array([])
    list_offset = range(0, 1015, 49)
    data_length = 49
    cir_buffer = Buffer([0] * 96 * len(list_offset), size=2, signed=1)
    p.getDeviceCir(list_offset, data_length, cir_buffer)
    try:
        import matplotlib.pyplot as plt
        import numpy as np
#         get real and imaginarypart of the cir buffer
        real = np.array(cir_buffer.data[0::2])
        imag = np.array(cir_buffer.data[1::2])
        # create an image of the CIR
        cira = real + 1j*imag
#       That plots the CIR contains in the buffer.
#       It still requires post-procesing to
#       re-align delay and received power level.
        plt.plot(20*np.log10(abs(cira[:-36])))
    except:
        print cir_buffer
        plt.show()
else:
    print 'Ranging failed'