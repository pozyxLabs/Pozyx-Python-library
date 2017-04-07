"""pypozyx.pozyx_serial - contains the serial interface with Pozyx through PozyxSerial."""
from time import sleep

from pypozyx.definitions.constants import *
from pypozyx.definitions.registers import *
from pypozyx.lib import PozyxLib
from pypozyx.structures.generic import Data, SingleRegister
from serial import Serial
from serial.tools.list_ports import comports

## \addtogroup auxiliary_serial
# @{

def list_serial_ports():
    """Prints the open serial ports line per line"""
    ports = comports()
    for port in ports:
        print(port)


def get_serial_ports():
    """Returns the open serial ports"""
    return comports()


def get_pozyx_ports():
    """Returns the Pozyx serial ports. Untested on UNIX"""
    ports = get_serial_ports()
    pozyx_ports = []
    for port in ports:
        if "STMicroelectronics Virtual COM Port" in port.description:
            pozyx_ports.append(port.device)

## @}


class PozyxSerial(PozyxLib):
    """
    PozyxSerial
    ===========

    This class provides the Pozyx Serial interface, and opens and locks the serial
    port to use with Pozyx. All functionality from PozyxLib and PozyxCore is included.

    Args:
        port: string name of the serial port. On UNIX this will be '/dev/ttyACMX', on
            Windows this will be 'COMX', with X a random number.

    Kwargs:
        baudrate: the baudrate of the serial port. Default value is 115200.
        timeout: timeout for the serial port communication in seconds. Default is 0.1s or 100ms.
        print_output: boolean for debugging purposes. If set to True,

    Example usage:
        >>> pozyx = PozyxSerial('COMX') # Windows
        >>> pozyx = PozyxSerial('/dev/ttyACMX', print_output=True) # Linux and OSX. Also puts debug output on.

    Finding the serial port
    =======================
    Finding the serial port can be easily done with the following code:
        >>> import serial.tools.list_ports
        >>> print serial.tools.list_ports.comports()[0]

    Putting one and two together, automating the correct port selection with one Pozyx attached:
        >>> import serial.tools.list_ports
        >>> pozyx = PozyxSerial(serial.tools.list_ports.comports()[0])
    """

    ## \addtogroup core
    # @{
    def __init__(self, port, baudrate=115200, timeout=0.1, write_timeout=0.1, print_output=False, debug_trace=False):
        """Initializes the PozyxSerial object. See above for details."""
        self.print_output = print_output
        try:
            self.ser = Serial(port, baudrate, timeout=timeout,
                              write_timeout=write_timeout)
        except:
            print(
                "Couldn't connect with Pozyx, wrong/busy serial port, or pySerial not installed.")
            if debug_trace:
                import traceback
                import sys
                traceback.print_tb(sys.exc_info()[2])
            raise SystemExit

        sleep(0.25)

        regs = Data([0, 0, 0])
        if self.regRead(POZYX_WHO_AM_I, regs) == POZYX_FAILURE:
            print("Connected to Pozyx, but couldn't read serial data.")
            if debug_trace:
                import traceback
                import sys
                traceback.print_tb(sys.exc_info()[2])
            raise SystemExit

        self._hw_version = regs[1]
        self._sw_version = regs[2]

        if regs[0] != 0x43:
            print("WHO AM I returned 0x%0.2x, something is wrong with Pozyx." %
                  regs[0])
            raise SystemExit

    ## @}

    def regWrite(self, address, data):
        """
        Writes data to the Pozyx registers, starting at a register address, if registers are writable.

        Args:
            address: Register address to start writing at.
            data: Data to write to the Pozyx registers. Has to be ByteStructure-derived object.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE
        """
        data.load_hex_string()
        index = 0
        runs = int(data.byte_size / MAX_SERIAL_SIZE)
        for i in range(runs):
            s = 'W,%0.2x,%s\r' % (
                address + index, data.byte_data[2 * index: 2 * (index + MAX_SERIAL_SIZE)])
            index += MAX_SERIAL_SIZE
            try:
                self.ser.write(s.encode())
            except:
                return POZYX_FAILURE
            # delay(POZYX_DELAY_LOCAL_WRITE)
        s = 'W,%0.2x,%s\r' % (address + index, data.byte_data[2 * index:])
        try:
            self.ser.write(s.encode())
        except:
            return POZYX_FAILURE
        return POZYX_SUCCESS

    def serialExchange(self, s):
        """
        Auxiliary. Performs a serial write to and read from the Pozyx.

        Args:
            s: Serial message to send to the Pozyx
        Returns:
            Serial message the Pozyx returns, stripped from 'D,' at its start
                and NL+CR at the end.
        """
        self.ser.write(s.encode())
        response = self.ser.readline().decode()
        if self.print_output:
            print('The response to %s is %s.' % (s.strip(), response.strip()))
        if len(response) == 0:
            raise EnvironmentError
        if response[0] == 'D':
            return response[2:-2]
        raise EnvironmentError

    def regRead(self, address, data):
        """
        Reads data from the Pozyx registers, starting at a register address, if registers are readable.

        Args:
            address: Register address to start writing at.
            data: Data to write to the Pozyx registers. Has to be ByteStructure-derived object.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE
        """
        runs = int(data.byte_size / MAX_SERIAL_SIZE)
        r = ''
        for i in range(runs):
            s = 'R,%0.2x,%i\r' % (
                address + i * MAX_SERIAL_SIZE, MAX_SERIAL_SIZE)
            try:
                r += self.serialExchange(s)
            except:
                return POZYX_FAILURE
        s = 'R,%0.2x,%i\r' % (
            address + runs * MAX_SERIAL_SIZE, data.byte_size - runs * MAX_SERIAL_SIZE)
        try:
            r += self.serialExchange(s)
        except:
            return POZYX_FAILURE
        data.load_bytes(r)
        return POZYX_SUCCESS

    def regFunction(self, address, params, data):
        """
        Performs a register function on the Pozyx, if the address is a register function.

        Args:
            address: Register function address of function to perform.
            params: Parameters for the register function. Has to be ByteStructure-derived object.
            data: Container for the data the register function returns. Has to be ByteStructure-derived object.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE
        """
        params.load_hex_string()
        s = 'F,%0.2x,%s,%i\r' % (address, params.byte_data, data.byte_size + 1)
        try:
            r = self.serialExchange(s)
        except:
            return POZYX_FAILURE
        if len(data) > 0:
            data.load_bytes(r[2:])
        return int(r[0:2], 16)

    def waitForFlag(self, interrupt_flag, timeout_s, interrupt=None):
        """
        Waits for a certain interrupt flag to be triggered, indicating that that type of interrupt
        occured.

        Args:
            interrupt_flag: Flag indicating interrupt type.
            timeout_s: time in seconds that POZYX_INT_STATUS will be checked for the flag before returning
                POZYX_TIMEOUT.

        Kwargs:
            interrupt: Container for the POZYX_INT_STATUS data. For debugging purposes.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        if interrupt is None:
            interrupt = SingleRegister()
        return self.waitForFlag_safe(interrupt_flag, timeout_s, interrupt)
