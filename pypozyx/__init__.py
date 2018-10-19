#!/usr/bin/env python
"""Provides:

- A way to interact with Pozyx over USB through PozyxSerial
- A solid set of easy-to-use and easy-to-expand functions for
  working with Pozyx.

NB: At this point, documentation is planned for functions and the structures
relevant for users. This constitutes the lib, structures.device, and
structures.sensor_data modules.

How to use the documentation
----------------------------
Documentation is available in two forms: docstrings provided with the code,
and the documentation available on `the Pozyx documentation <https://www.pozyx.io/Documentation/Datasheet/arduino>`_.
For now, this links to the Arduino library functions, as there is a lot of
similarity and evidently both libraries influence each other. If you're not
working low-level, you shouldn't notice much difference between the two.
When the Python webpage comes only, we'll clearly distinguish between
functions not available in the Arduino lib.

The docstring examples assume that, for ease of use and typing, everything
from the pypozyx package has been imported using::

  >>> from pypozyx import *

While not considered good practice, it makes reading and writing scripts
much easier when starting out.

This imports PozyxSerial, all constants definitions, and the device, generic,
and sensordata structures. This does not import the registers definitions,
or the bitmasks, as these are considered advanced features and assume the
user knows what he's doing. To access these, import the modules
pypozyx.definitions.bitmasks and pypozyx.definitions.registers.

Whenever you see the following code in the docstrings::

  >>> pozyx.anyFunction()
  >>> # or
  >>> p.anyFunction()

'pozyx' and 'p' are the actual Pozyx device's interface instance, not a module.
This means that the Pozyx interface instance performs anyFunction.

Available subpackages
---------------------
structures
    Basic and specialised data structures tailored to using Pozyx
definitions
    Pozyx constant definitions, such as general constants, register indexes,
    and bit masks for certain functionality.


Serial port warnings
--------------------
Make  sure that the serial connection is closed (i.e. program terminated)
before disconnecting Pozyx. Terminating a running program can be done using
Ctrl+Z or Ctrl+C. If something goes wrong, chances are high you will have to
edit which serial port is used by Pozyx, as the previous one will be
considered unavailable by your OS.

Automatically selecting serial port
-----------------------------------
A trick to remedy this is by using the following code::

  >>> import serial.tools.list_ports
  >>> port = list(serial.tools.list_ports.comports())[0]

This only works consistently when Pozyx is the only serial device you're
working with, when you use more devices, their order in the list may
switch at some point.

"""

__version__ = '1.2.4'

VERSION = __version__
version = __version__


from pypozyx.definitions import *
from pypozyx.pozyx_serial import *
from pypozyx.structures.device import *
from pypozyx.structures.generic import *
from pypozyx.structures.sensor_data import *
