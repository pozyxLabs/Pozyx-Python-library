#!/usr/bin/env python
"""
pypozyx.structures - contains ByteStructure and derived classes that contain the Pozyx data.

The PyPozyx library is designed to work with these objects, and they abstract the low-level
approach that would otherwise be necessary in using the serial port, so these are very
recommended to use.

This package is subdivided in three groups of structures, derived from ByteStructure:
    - device: containers for device functionality. Network ID, UWB settings, device coordinates/range
    - sensor_data: containers for sensor data retrieved from the Pozyx. Magnetic, acceleration, etc.
        These also take care of the physical convertion to the respective standard units for that sensor.
    - generic: as the name suggests, generic containers. The SingleRegister object is a
        multipurpose object that can be used to read single registers of any size. Data can be
        used to create your own arbitrary packed structures.

When importing this package, you will get all of the device classes, the sensor data classes,
and both SingleRegister and Data.
"""
from pypozyx.structures.device import *
from pypozyx.structures.sensor_data import *
from pypozyx.structures.generic import SingleRegister, Data
