"""pypozyx.pozyx_i2c - contains the I2C interface with Pozyx through PozyxI2C."""
from pypozyx.core import PozyxConnectionError

from pypozyx.definitions.constants import (POZYX_SUCCESS, POZYX_FAILURE,
                                           PozyxConstants)

from pypozyx.lib import PozyxLib
from pypozyx.structures.generic import SingleRegister
from smbus2 import SMBus, i2c_msg

from warnings import warn


class PozyxI2C(PozyxLib):
    """This class provides the Pozyx I2C interface.
    All functionality from PozyxLib and PozyxCore is included.

    Args:
        bus: i2c bus number (e.g. 0 or 1) or an absolute file path (e.g. `/dev/i2c-42`).
        print_output (optional): boolean for printing the serial exchanges, mainly for debugging purposes
        suppress_warnings (optional): boolean for suppressing warnings in the Pozyx use, usage not recommended
        debug_trace (optional): boolean for printing the trace on bad serial init (DEPRECATED)
        show_trace (optional): boolean for printing the trace on bad serial init (DEPRECATED)
    """

    def __init__(self, bus=1,
                 print_output=False, debug_trace=False, show_trace=False,
                 suppress_warnings=False):
        """Initializes the PozyxI2C object. See above for details."""
        super(PozyxI2C, self).__init__()
        self.print_output = print_output
        if debug_trace is True or show_trace is True:
            if not suppress_warnings:
                warn("debug_trace or show_trace are on their way out, exceptions of the type PozyxException are now raised.",
                     DeprecationWarning)
        self.suppress_warnings = suppress_warnings

        self.bus = SMBus(bus)

        self.validatePozyx()

    def validatePozyx(self):
        """Validates whether the connected device is indeed a Pozyx device"""
        whoami = SingleRegister()
        if self.getWhoAmI(whoami) != POZYX_SUCCESS:
            raise PozyxConnectionError("Connected to device, but couldn't read I2C data. Is it a Pozyx?")
        if whoami.value != 0x43:
            raise PozyxConnectionError("POZYX_WHO_AM_I returned 0x%0.2x, something is wrong with Pozyx." % whoami.value)

    def regWrite(self, address, data):
        """
        Writes data to the Pozyx registers, starting at a register address,
        if registers are writable.

        Args:
            address: Register address to start writing at.
            data: Data to write to the Pozyx registers.
                Has to be ByteStructure-derived object.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE
        """
        bdata = data.transform_to_bytes()
        try:
            msg_addr  = i2c_msg.write(PozyxConstants.POZYX_I2C_ADDRESS, bytes([address]))
            msg_write = i2c_msg.write(PozyxConstants.POZYX_I2C_ADDRESS, bdata)
            self.bus.i2c_rdwr(msg_addr, msg_write)
        except OSError:
            return POZYX_FAILURE
        return POZYX_SUCCESS

    def regRead(self, address, data):
        """
        Reads data from the Pozyx registers, starting at a register address,
        if registers are readable.

        Args:
            address: Register address to start writing at.
            data: Data to read from the Pozyx registers.
                Has to be ByteStructure-derived object.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE
        """
        try:
            msg_addr = i2c_msg.write(PozyxConstants.POZYX_I2C_ADDRESS, bytes([address]))
            msg_read = i2c_msg.read(PozyxConstants.POZYX_I2C_ADDRESS, data.byte_size)
            self.bus.i2c_rdwr(msg_addr, msg_read)
            r = bytes(msg_read)
        except OSError:
            return POZYX_FAILURE
        data.load_packed(r)
        return POZYX_SUCCESS

    def regFunction(self, address, params, data):
        """
        Performs a register function on the Pozyx, if the address is a register
        function.

        Args:
            address: Register function address of function to perform.
            params: Parameters for the register function.
                Has to be ByteStructure-derived object.
            data: Container for the data the register function returns.
                Has to be ByteStructure-derived object.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE
        """
        bparams = params.transform_to_bytes()
        try:
            msg_write = i2c_msg.write(PozyxConstants.POZYX_I2C_ADDRESS, bytes([address]) + bytes(bparams))
            msg_read  = i2c_msg.read(PozyxConstants.POZYX_I2C_ADDRESS, data.byte_size + 1)
            self.bus.i2c_rdwr(msg_write, msg_read)
            r = bytes(msg_read)
        except OSError:
            return POZYX_FAILURE
        if len(data) > 0:
            data.load_packed(r[1:])
        return r[0]

    def waitForFlag(self, interrupt_flag, timeout_s, interrupt):
        """
        Waits for a certain interrupt flag to be triggered, indicating that
        that type of interrupt occured.

        Args:
            interrupt_flag: Flag indicating interrupt type.
            timeout_s: time in seconds that POZYX_INT_STATUS will be checked
                for the flag before returning POZYX_TIMEOUT.

        Kwargs:
            interrupt: Container for the POZYX_INT_STATUS data

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        if interrupt is None:
            interrupt = SingleRegister()
        return self.waitForFlagSafe(interrupt_flag, timeout_s, interrupt)
