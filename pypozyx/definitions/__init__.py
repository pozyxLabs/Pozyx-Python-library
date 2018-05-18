#!/usr/bin/env python
"""
pypozyx.definitions - contains all Pozyx constant definitions.

These definitions are divided in three large groups:
    - constants: physical constants, type definitions...
    - bitmasks: bitmasks used for ANDing register data against, especially useful for interrupt status data.
    - registers: definitions of the register addresses, for more 'advanced' use.

When importing this, only the constants will get imported. To import the registers and bitmasks,
use the following code:
    >>> from pypozyx.definitions.registers import * # or specific registers for best practice
    >>> from pypozyx.definitions.bitmasks import * # or specific bitmasks for best practice
"""
from pypozyx.definitions.constants import *
from pypozyx.definitions.bitmasks import PozyxBitmasks
from pypozyx.definitions.registers import PozyxRegisters
