#!/usr/bin/env python
"""pypozyx.core - core Pozyx interface and inter-Pozyx communication functionality through the PozyxCore class"""
from time import sleep, time

from pypozyx.definitions.bitmasks import (POZYX_INT_STATUS_ERR,
                                          POZYX_INT_STATUS_FUNC)
from pypozyx.definitions.constants import (MAX_BUF_SIZE, MAX_SERIAL_SIZE,
                                           POZYX_DELAY_LOCAL_WRITE,
                                           POZYX_DELAY_POLLING,
                                           POZYX_DELAY_REMOTE_WRITE,
                                           POZYX_FAILURE, POZYX_SUCCESS,
                                           POZYX_TIMEOUT)
from pypozyx.definitions.registers import (POZYX_INT_STATUS, POZYX_RX_DATA,
                                           POZYX_RX_NETWORK_ID, POZYX_TX_DATA,
                                           POZYX_TX_SEND)
from pypozyx.structures.generic import (Data, SingleRegister, dataCheck,
                                        is_functioncall, is_reg_readable,
                                        is_reg_writable)

from warnings import warn

class PozyxCore():
    """PozyxCore

    PozyxCore
    =========

    Implements virtual core Pozyx interfacing functions such as regRead,
    regWrite and regFunction, which have to be implemented in the derived interface.
    Auxiliary functions for core functionality, getRead, setWrite, useFunction,
    and checkForFlag, are also included in PozyxCore.

    waitForFlag_safe, which uses polling, is also implemented interface-independently.

    Apart from these core interface functions, core inter-Pozyx communication
    functionality is also implemented here. This includes remote interface functions
    and general communication to send and receive data from other devices.
    """

    def __init__(self):
        """
        Constructor for PozyxCore.

        PozyxCore isn't practically usable on its own as it misses a core interface
        implementation.
        """
        pass

    def regRead(self, address, data):
        """
        Stores the read amount of bytes equal to data's size starting at address into data.

        This is a virtual function, be sure to implement this in your own interface.
        """
        raise NotImplementedError(
            'You need to override this function in your derived interface!')

    def regWrite(self, address, data):
        """
        Writes the given data starting at address.

        This is a virtual function, be sure to implement this in your own interface.
        """
        raise NotImplementedError(
            'You need to override this function in your derived interface!')

    def regFunction(self, address, params, data):
        """
        Performs a function with given parameters, storing its output in data.

        This is a virtual function, be sure to implement this in your own interface.
        """
        raise NotImplementedError(
            'You need to override this function in your derived interface!')

    def remoteRegWrite(self, destination, address, data):
        """
        Performs regWrite on a remote Pozyx device.

        Args:
            destination: Network ID of destination device. integer ID or NetworkID(ID).
            address: Register address to start the writing operation on.
            data: Contains the data to be written. ByteStructure-derived object.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        if len(data) > MAX_BUF_SIZE - 1:
            return POZYX_FAILURE

        send_data = Data([0, address] + data.data, 'BB' + data.data_format)
        status = self.regFunction(POZYX_TX_DATA, send_data, Data([]))
        if status == POZYX_FAILURE:
            return status

        self.regRead(POZYX_INT_STATUS, SingleRegister())
        params = Data([destination, 0x04], 'HB')
        status = self.regFunction(POZYX_TX_SEND, params, Data([]))
        if status == POZYX_FAILURE:
            return status
        return self.checkForFlag(POZYX_INT_STATUS_FUNC, 0.5)

    def remoteRegRead(self, destination, address, data):
        """
        Performs regRead on a remote Pozyx device.

        Args:
            destination: Network ID of destination device. integer ID or NetworkID(ID).
            address: Register address to start the read operation from.
            data: Container for the read data. ByteStructure-derived object.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        if dataCheck(destination):
            destination = destination[0]
        if len(data) > MAX_BUF_SIZE:
            return POZYX_FAILURE
        if destination == 0:
            return POZYX_FAILURE

        send_data = Data([0, address, data.byte_size])
        status = self.regFunction(POZYX_TX_DATA, send_data, Data([]))
        if status == POZYX_FAILURE:
            return status

        self.regRead(POZYX_INT_STATUS, SingleRegister())
        params = Data([destination, 0x02], 'HB')
        status = self.regFunction(POZYX_TX_SEND, params, Data([]))
        if status == POZYX_FAILURE:
            return status

        status = self.checkForFlag(POZYX_INT_STATUS_FUNC, 1)
        if status == POZYX_SUCCESS:
            rx_info = Data([0, 0], 'HB')
            self.regRead(POZYX_RX_NETWORK_ID, rx_info)
            if rx_info[0] == destination and rx_info[1] == data.byte_size:
                status = self.readRXBufferData(data)
                return status
            else:
                return POZYX_FAILURE
        return status

    def remoteRegFunction(self, destination, address, params, data):
        """
        Performs regFunction on a remote Pozyx device.

        Args:
            destination: Network ID of destination device. integer ID or NetworkID(ID).
            address: Register address to start the read operation from.
            params: Parameters for the register function. ByteStructure-derived object of uint8s.
            data: Container for the data returned by the register function. ByteStructure-derived object.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        send_data = Data([0, address] + params.data, 'BB' + params.data_format)
        status = self.regFunction(POZYX_TX_DATA, send_data, Data([]))
        if status == POZYX_FAILURE:
            return status

        self.regRead(POZYX_INT_STATUS, SingleRegister())
        tx_params = Data([destination, 0x08], 'HB')
        status = self.regFunction(POZYX_TX_SEND, tx_params, Data([]))
        if status == POZYX_FAILURE:
            return status

        status = self.checkForFlag(POZYX_INT_STATUS_FUNC, 1)
        if status == POZYX_SUCCESS:
            rx_info = Data([0, 0], 'HB')
            self.regRead(POZYX_RX_NETWORK_ID, rx_info)
            if rx_info[0] == destination and rx_info[1] == data.byte_size + 1:
                return_data = Data([0] + data.data, 'B' + data.data_format)
                status = self.readRXBufferData(return_data)
                if status == POZYX_FAILURE:
                    return status
                if len(return_data) > 1:
                    data.load(return_data[1:])
                return return_data[0]
            else:
                return POZYX_FAILURE
        return status

    def waitForFlag(self, interrupt_flag, timeout_s, interrupt=None):
        """
        Checks the interrupt register for given flag until encountered/past the timeout time.

        This is a virtual function, be sure to implement this in your derived interface.
        """
        raise NotImplementedError(
            'You need to override this function in your derived interface!')

    def waitForFlag_safe(self, interrupt_flag, timeout_s, interrupt=None):
        """
        Performs waitForFlag in polling mode.

        Args:
            interrupt_flag: Flag of interrupt type to check the interrupt register against.
            timeout_s: duration to wait for the interrupt in seconds.

        Kwags:
            interrupt: Container for the interrupt status register data.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        if interrupt is None:
            interrupt = SingleRegister()
        start = time()
        while(time() - start < timeout_s):
            sleep(POZYX_DELAY_POLLING)
            status = self.regRead(POZYX_INT_STATUS, interrupt)
            if (interrupt[0] & interrupt_flag) and status == POZYX_SUCCESS:
                return True
        return False

    ## \addtogroup core
    # @{

    def setWrite(self, address, data, remote_id=None, local_delay=POZYX_DELAY_LOCAL_WRITE, remote_delay=POZYX_DELAY_REMOTE_WRITE):
        """
        Writes data to Pozyx registers either locally or remotely.

        Args:
            address: The register address
            data: A ByteStructure - derived object that contains the data to be written.

        Kwargs:
            remote_id: Remote ID for remote read.
            local_delay: Delay after a local write
            remote_delay: Delay after a remote write

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT

        Examples:
            >>> leds = SingleRegister(0xFF)
            >>> self.setWrite(POZYX_LED_CTRL, leds)
        """
        if not is_reg_writable(address):
            if not self.suppress_warnings:
                warn("Register 0x%0.02x isn't writable" % address, stacklevel=3)
        if remote_id is None:
            status = self.regWrite(address, data)
            sleep(local_delay)
        else:
            status = self.remoteRegWrite(remote_id, address, data)
            sleep(remote_delay)
        return status

    def getRead(self, address, data, remote_id=None):
        """
        Reads Pozyx register data either locally or remotely.

        Args:
            address: The register address
            data: A ByteStructure - derived object that is the container of the read data.

        Kwargs:
            remote_id: Remote ID for remote read.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT

        Example:
            >>> whoami = SingleRegister()
            >>> self.getRead(POZYX_WHO_AM_I, whoami)
            >>> print(whoami)
            67
        """
        if not is_reg_readable(address):
            if not self.suppress_warnings:
                warn("Register 0x%0.02x isn't readable" % address, stacklevel=3)
        if remote_id is None:
            return self.regRead(address, data)
        else:
            return self.remoteRegRead(remote_id, address, data)

    def useFunction(self, function, params=None, data=None, remote_id=None):
        """
        Activates a Pozyx register function either locally or remotely.

        Args:
            address: The function address

        Kwargs:
            params: A ByteStructure - derived object that contains the parameters for the function.
            data: A ByteStructure - derived object that is the container of the read data.
            remote_id: Remote ID for remote read.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT

        Example:
            >>> self.useFunction(POZYX_DEVICES_CLEAR)
        """
        if not is_functioncall(function):
            if not self.suppress_warnings:
                warn("Register 0x%0.02x isn't a function register" % address, stacklevel=3)
        if params is None:
            params = Data([])
        if data is None:
            data = Data([])
        if remote_id is None:
            status = self.regFunction(function, params, data)
        else:
            status = self.remoteRegFunction(
                remote_id, function, params, data)
        return status

    # wait for flag functions

    def checkForFlag(self, interrupt_flag, timeout_s, interrupt=None):
        """Performs waitForFlag_safe and checks against errors or timeouts.

        This abstracts the waitForFlag status check routine commonly encountered
        in more complex library functions and checks the given flag against
        the error flag.

        Args:
            interrupt_flag: Flag of interrupt type to check the interrupt register against.
            timeout_s: duration to wait for the interrupt in seconds

        Kwags:
            interrupt: Container for the interrupt status register data.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        if interrupt is None:
            interrupt = SingleRegister()
        if self.waitForFlag_safe(interrupt_flag | POZYX_INT_STATUS_ERR, timeout_s, interrupt):
            if (interrupt[0] & POZYX_INT_STATUS_ERR) == POZYX_INT_STATUS_ERR:
                return POZYX_FAILURE
            else:
                return POZYX_SUCCESS
        else:
            return POZYX_TIMEOUT

    ## @}

    ## \addtogroup communication_functions

    def readRXBufferData(self, data):
        """
        Reads the device's receive buffer's data completely.

        Args:
            data: Container for the data to be read from the receiver buffer.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE
        """
        if data.byte_size > MAX_BUF_SIZE:
            return POZYX_FAILURE

        status = POZYX_SUCCESS
        if data.byte_size < MAX_SERIAL_SIZE:
            params = Data([0, data.byte_size])
            status &= self.regFunction(POZYX_RX_DATA, params, data)
        else:
            runs = int(data.byte_size / MAX_SERIAL_SIZE)
            s = ''
            for i in range(runs):
                params = Data([i * MAX_SERIAL_SIZE, MAX_SERIAL_SIZE])
                d = Data([0] * MAX_SERIAL_SIZE)
                status &= self.regFunction(POZYX_RX_DATA, params, d)
                s += d.byte_data
            params = Data(
                [runs * MAX_SERIAL_SIZE, data.byte_size - runs * MAX_SERIAL_SIZE])
            d = Data([0] * (data.byte_size - runs * MAX_SERIAL_SIZE))
            status &= self.regFunction(POZYX_RX_DATA, params, d)
            s += d.byte_data
            data.load_bytes(s)
        return status

    def writeTXBufferData(self, data, offset=0):
        """
        Writes data to the device's transmit buffer at the offset address.

        Args:
            data: Data to write to the Pozyx buffer. Has to be a ByteStructure derived object.

        Kwargs:
            offset: Offset in buffer to start writing data

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE
        """
        if offset + data.byte_size > MAX_BUF_SIZE:
            return POZYX_FAILURE

        # have to account for the parameter taking up a byte
        _MAX_SERIAL_SIZE = MAX_SERIAL_SIZE - 1

        status = POZYX_SUCCESS
        data = Data(data.transform_to_bytes())
        runs = int(data.byte_size / _MAX_SERIAL_SIZE)
        for i in range(runs):
            params = Data([i * _MAX_SERIAL_SIZE] + data[i *
                                                        _MAX_SERIAL_SIZE: (i + 1) * _MAX_SERIAL_SIZE])
            status &= self.regFunction(POZYX_TX_DATA, params, Data([]))
        params = Data([runs * _MAX_SERIAL_SIZE] +
                      data[runs * _MAX_SERIAL_SIZE:])
        return status & self.regFunction(POZYX_TX_DATA, params, Data([]))

    def sendTXBufferData(self, destination):
        """
        Sends the transmit buffer's data to the destination device.

        Args:
            destination: Network ID of destination. integer ID or NetworkID(ID)

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE
        """
        if dataCheck(destination):
            destination = destination[0]  # see what I did there.
        params = Data([destination, 0x06], 'HB')
        status = self.regFunction(POZYX_TX_SEND, params, Data([]))
        return status

    def sendData(self, destination, data):
        """
        Stores the data in the transmit buffer and then sends it to the device with ID destination.

        Args:
            destination: Network ID of destination. integer ID or NetworkID(ID)
            data: Data to send to the destination. Has to be a ByteStructure derived object.

        Performs the following code::

          >>>self.writeTXBufferData(data)
          >>>self.sendTXBufferData(destination)

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE
        """
        status = POZYX_SUCCESS
        status &= self.writeTXBufferData(data)
        status &= self.sendTXBufferData(destination)
        return status

    ## @}
