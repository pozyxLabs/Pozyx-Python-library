#!/usr/bin/env python
"""

"""

from pypozyx.lib import PozyxLib
from pypozyx.definitions.constants import MODE_POLLING
# from pypozyx.definitions.registers import *


class PozyxI2C(PozyxLib):

    def __init__(self, mode=MODE_POLLING, print_output=False):
        pass

    def regWrite(self, address, data):
        pass

    def regRead(self, address, data):
        pass

    def regFunction(self, address, params, data):
        pass

    #
    def waitForFlag(self, interrupt_flag, timeout_ms, interrupt):
        pass

    #
    def waitForFlagSafe(self, interrupt_flag, timeout_ms, interrupt):
        pass

    def configInterruptPin(self, pin, mode, bActiveHigh, bLatch, remote_id=None):
        pass
