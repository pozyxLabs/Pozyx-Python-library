"""pypozyx.pozyx_serial - contains the serial interface with Pozyx through PozyxSerial."""
from time import sleep
from pypozyx.core import PozyxConnectionError

from pypozyx.definitions.constants import (POZYX_SUCCESS, POZYX_FAILURE,
                                           MAX_SERIAL_SIZE)

from pypozyx.lib import PozyxLib
from pypozyx.structures.generic import SingleRegister
from serial import Serial, VERSION as PYSERIAL_VERSION, SerialException
from serial.tools.list_ports import comports

from warnings import warn

# \addtogroup auxiliary_serial
# @{


def list_serial_ports():
    """Prints the open serial ports line per line"""
    warn("list_serial_ports now deprecated, use print_all_serial_ports instead", DeprecationWarning)
    ports = comports()
    for port in ports:
        print(port)


def print_all_serial_ports():
    """Prints the open serial ports line per line"""
    ports = comports()
    for port in ports:
        print(port)


def get_serial_ports():
    """Returns the open serial ports"""
    return comports()


def is_pozyx_port(port):
    """Returns whether the port is a Pozyx device"""
    try:
        if "Pozyx Labs" in port.manufacturer:
            return True
    except TypeError:
        pass
    try:
        if "Pozyx" in port.product:
            return True
    except TypeError:
        pass
    try:
        if "0483:" in port.hwid:
            return True
    except TypeError:
        pass
    return False


def get_port_object(device):
    """Returns the PySerial port object from a given port path"""
    for port in get_serial_ports():
        if port.device == device:
            return port


def is_pozyx(device):
    """Returns whether the device is a recognized Pozyx device"""
    port = get_port_object(device)
    if port is not None and is_pozyx_port(port):
        return True
    return False


def get_pozyx_ports():
    """Returns the Pozyx serial ports. Windows only. Needs driver installed"""
    pozyx_ports = []
    for port in get_serial_ports():
        if is_pozyx_port(port):
            pozyx_ports.append(port.device)
    return pozyx_ports


def get_first_pozyx_serial_port():
    """Returns the first encountered Pozyx serial port's identifier"""
    for port in get_serial_ports():
        if is_pozyx_port(port):
            return port.device


def get_pozyx_ports_windows():
    """Returns the Pozyx serial ports. Windows only. Needs driver installed"""
    ports = get_serial_ports()
    pozyx_ports = []
    for port in ports:
        if "STMicroelectronics Virtual COM Port" in port.description:
            pozyx_ports.append(port.device)


def is_correct_pyserial_version():
    """Returns whether the pyserial version is supported"""
    version_tags = [int(version_tag)
                    for version_tag in PYSERIAL_VERSION.split('.')]
    if version_tags[0] >= 3:
        if version_tags[0] == 3 and version_tags[1] <= 3:
            warn("PySerial out of date, please update to v3.4 if possible", stacklevel=0)
        return True
    return False


# @}


class PozyxSerial(PozyxLib):
    """This class provides the Pozyx Serial interface, and opens and locks the serial
    port to use with Pozyx. All functionality from PozyxLib and PozyxCore is included.

    Args:
        port (str): Name of the serial port. On UNIX this will be '/dev/ttyACMX', on
            Windows this will be 'COMX', with X a random number.
        baudrate (optional): the baudrate of the serial port. Default value is 115200.
        timeout (optional): timeout for the serial port communication in seconds. Default is 0.1s or 100ms.
        print_output (optional): boolean for printing the serial exchanges, mainly for debugging purposes
        suppress_warnings (optional): boolean for suppressing warnings in the Pozyx use, usage not recommended
        debug_trace (optional): boolean for printing the trace on bad serial init (DEPRECATED)
        show_trace (optional): boolean for printing the trace on bad serial init (DEPRECATED)

    Example usage:
        >>> pozyx = PozyxSerial('COMX') # Windows
        >>> pozyx = PozyxSerial('/dev/ttyACMX', print_output=True) # Linux and OSX. Also puts debug output on.

    Finding the serial port can be easily done with the following code:
        >>> import serial.tools.list_ports
        >>> print serial.tools.list_ports.comports()[0]

    Putting one and two together, automating the correct port selection with one Pozyx attached:
        >>> import serial.tools.list_ports
        >>> pozyx = PozyxSerial(serial.tools.list_ports.comports()[0])
    """

    # \addtogroup core
    # @{
    def __init__(self, port, baudrate=115200, timeout=0.1, write_timeout=0.1,
                 print_output=False, debug_trace=False, show_trace=False,
                 suppress_warnings=False):
        """Initializes the PozyxSerial object. See above for details."""
        super(PozyxSerial, self).__init__()
        self.print_output = print_output
        if debug_trace is True or show_trace is True:
            if not suppress_warnings:
                warn("debug_trace or show_trace are on their way out, exceptions of the type PozyxException are now raised.",
                     DeprecationWarning)
        self.suppress_warnings = suppress_warnings

        self.connectToPozyx(port, baudrate, timeout, write_timeout)

        sleep(0.25)

        self.validatePozyx()

    def connectToPozyx(self, port, baudrate, timeout, write_timeout):
        """Attempts to connect to the Pozyx via a serial connection"""
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.write_timeout = write_timeout

        try:
            if is_correct_pyserial_version():
                if not is_pozyx(port) and not self.suppress_warnings:
                    warn("The passed device is not a recognized Pozyx device, is %s" % get_port_object(port).description, stacklevel=2)
                self.ser = Serial(port=port, baudrate=baudrate, timeout=timeout,
                                  write_timeout=write_timeout)
            else:
                if not self.suppress_warnings:
                    warn("PySerial version %s not supported, please upgrade to 3.0 or (prefferably) higher" %
                         PYSERIAL_VERSION, stacklevel=0)
                self.ser = Serial(port=port, baudrate=baudrate, timeout=timeout,
                                  writeTimeout=write_timeout)
        except SerialException as exc:
            raise PozyxConnectionError("Wrong or busy serial port, SerialException: {}".format(str(exc)))
        except Exception as exc:
            raise PozyxConnectionError("Couldn't connect to Pozyx, {}: {}".format(exc.__class__.__name__, str(exc)))

    def validatePozyx(self):
        """Validates whether the connected device is indeed a Pozyx device"""
        whoami = SingleRegister()
        if self.getWhoAmI(whoami) != POZYX_SUCCESS:
            raise PozyxConnectionError("Connected to device, but couldn't read serial data. Is it a Pozyx?")
        if whoami.value != 0x43:
            raise PozyxConnectionError("POZYX_WHO_AM_I returned 0x%0.2x, something is wrong with Pozyx." % whoami.value)

    # @}

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
        data.load_hex_string()
        index = 0
        runs = int(data.byte_size / MAX_SERIAL_SIZE)
        for i in range(runs):
            s = 'W,%0.2x,%s\r' % (
                address + index, data.byte_data[2 * index: 2 * (index + MAX_SERIAL_SIZE)])
            index += MAX_SERIAL_SIZE
            try:
                self.ser.write(s.encode())
            except SerialException:
                return POZYX_FAILURE
            # delay(POZYX_DELAY_LOCAL_WRITE)
        s = 'W,%0.2x,%s\r' % (address + index, data.byte_data[2 * index:])
        try:
            self.ser.write(s.encode())
        except SerialException:
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
            raise SerialException
        if response[0] == 'D':
            return response[2:-2]
        raise SerialException

    def regRead(self, address, data):
        """
        Reads data from the Pozyx registers, starting at a register address,
        if registers are readable.

        Args:
            address: Register address to start writing at.
            data: Data to write to the Pozyx registers.
                Has to be ByteStructure-derived object.

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
            except SerialException:
                return POZYX_FAILURE
        s = 'R,%0.2x,%i\r' % (
            address + runs * MAX_SERIAL_SIZE, data.byte_size - runs * MAX_SERIAL_SIZE)
        try:
            r += self.serialExchange(s)
        except SerialException:
            return POZYX_FAILURE
        data.load_bytes(r)
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
        params.load_hex_string()
        s = 'F,%0.2x,%s,%i\r' % (address, params.byte_data, data.byte_size + 1)
        try:
            r = self.serialExchange(s)
        except SerialException:
            return POZYX_FAILURE
        if len(data) > 0:
            data.load_bytes(r[2:])
        return int(r[0:2], 16)

    def waitForFlag(self, interrupt_flag, timeout_s, interrupt=None):
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
