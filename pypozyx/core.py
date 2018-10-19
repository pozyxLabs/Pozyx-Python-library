#!/usr/bin/env python
"""pypozyx.core - core Pozyx interface and inter-Pozyx communication functionality through the PozyxCore class"""
from time import sleep, time

from pypozyx.definitions.bitmasks import PozyxBitmasks
from pypozyx.definitions.constants import PozyxConstants, POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
from pypozyx.definitions.registers import PozyxRegisters
from pypozyx.structures.generic import (Data, SingleRegister, dataCheck, is_functioncall,
                                        is_reg_readable, is_reg_writable)
from pypozyx.structures.device import RXInfo, TXInfo

from warnings import warn


class PozyxCore(object):
    """Implements virtual core Pozyx interfacing functions such as regRead,
    regWrite and regFunction, which have to be implemented in the derived interface.
    Auxiliary functions for core functionality, getRead, setWrite, useFunction,
    and checkForFlag, are also included in PozyxCore.

    waitForFlag_safe, which uses polling, is also implemented interface-independently.

    Apart from these core interface functions, core inter-Pozyx communication
    functionality is also implemented here. This includes remote interface functions
    and general communication to send and receive data from other devices.
    """

    def __init__(self):
        """Constructor for PozyxCore.

        PozyxCore isn't practically usable on its own as it misses a core interface
        implementation.
        """
        pass

    def regRead(self, address, data):
        """Stores the read amount of bytes equal to data's size starting at address into data.

        This is a virtual function, be sure to implement this in your own interface.
        """
        raise NotImplementedError(
            'You need to override this function in your derived interface!')

    def regWrite(self, address, data):
        """Writes the given data starting at address.

        This is a virtual function, be sure to implement this in your own interface.
        """
        raise NotImplementedError(
            'You need to override this function in your derived interface!')

    def regFunction(self, address, params, data):
        """Performs a function with given parameters, storing its output in data.

        This is a virtual function, be sure to implement this in your own interface.
        """
        raise NotImplementedError(
            'You need to override this function in your derived interface!')

    def remoteRegWrite(self, destination, address, data):
        """Performs regWrite on a remote Pozyx device.

        Args:
            destination: Network ID of destination device. integer ID or NetworkID(ID).
            address: Register address to start the writing operation on.
            data: Contains the data to be written. ByteStructure-derived object.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        if len(data) > PozyxConstants.MAX_BUF_SIZE - 1:
            return POZYX_FAILURE

        send_data = Data([0, address] + data.data, 'BB' + data.data_format)
        status = self.regFunction(PozyxRegisters.WRITE_TX_DATA, send_data, Data([]))
        if status != POZYX_SUCCESS:
            return status

        self.getInterruptStatus(SingleRegister())
        status = self.sendTXWrite(destination)
        if status != POZYX_SUCCESS:
            return status
        return self.checkForFlag(PozyxBitmasks.INT_STATUS_FUNC, 0.5)

    def remoteRegRead(self, destination, address, data):
        """Performs regRead on a remote Pozyx device.

        Args:
            destination: Network ID of destination device. integer ID or NetworkID(ID).
            address: Register address to start the read operation from.
            data: Container for the read data. ByteStructure-derived object.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        if dataCheck(destination):
            destination = destination[0]
        if len(data) > PozyxConstants.MAX_BUF_SIZE:
            return POZYX_FAILURE
        if destination == 0:
            return POZYX_FAILURE

        send_data = Data([0, address, data.byte_size])
        status = self.regFunction(PozyxRegisters.WRITE_TX_DATA, send_data, Data([]))
        if status != POZYX_SUCCESS:
            return status

        self.getInterruptStatus(SingleRegister())
        status = self.sendTXRead(destination)
        if status != POZYX_SUCCESS:
            return status

        status = self.checkForFlag(PozyxBitmasks.INT_STATUS_FUNC, 1)
        if status == POZYX_SUCCESS:
            rx_info = RXInfo()
            self.getRxInfo(rx_info)
            if rx_info.remote_id == destination and rx_info.amount_of_bytes == data.byte_size:
                status = self.readRXBufferData(data)
                return status
            else:
                return POZYX_FAILURE
        return status

    def remoteRegFunction(self, destination, address, params, data):
        """Performs regFunction on a remote Pozyx device.

        Args:
            destination: Network ID of destination device. integer ID or NetworkID(ID).
            address: Register address to start the read operation from.
            params: Parameters for the register function. ByteStructure-derived object of uint8s.
            data: Container for the data returned by the register function. ByteStructure-derived object.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        send_data = Data([0, address] + params.data, 'BB' + params.data_format)
        status = self.regFunction(PozyxRegisters.WRITE_TX_DATA, send_data, Data([]))
        if status != POZYX_SUCCESS:
            return status

        self.getInterruptStatus(SingleRegister())
        status = self.sendTXFunction(destination)
        if status != POZYX_SUCCESS:
            return status

        status = self.checkForFlag(PozyxBitmasks.INT_STATUS_FUNC, 1)
        if status == POZYX_SUCCESS:
            rx_info = RXInfo()
            self.getRxInfo(rx_info)
            if rx_info.remote_id == destination and rx_info.amount_of_bytes == data.byte_size + 1:
                return_data = Data([0] + data.data, 'B' + data.data_format)
                status = self.readRXBufferData(return_data)
                if status != POZYX_SUCCESS:
                    return status
                if len(return_data) > 1:
                    data.load(return_data[1:])
                return return_data[0]
            else:
                return POZYX_FAILURE
        return status

    def waitForFlag(self, interrupt_flag, timeout_s, interrupt=None):
        """Checks the interrupt register for given flag until encountered/past the timeout time.

        This is a virtual function, be sure to implement this in your derived interface.
        """
        raise NotImplementedError(
            'You need to override this function in your derived interface!')

    def waitForFlag_safe(self, interrupt_flag, timeout_s, interrupt=None):
        """Performs waitForFlag in polling mode.

        Args:
            interrupt_flag: Flag of interrupt type to check the interrupt register against.
            timeout_s: duration to wait for the interrupt in seconds.
            interrupt (optional): Container for the interrupt status register data.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        if not self.suppress_warnings:
            warn("waitForFlag_safe is deprecated, use waitForFlagSafe instead")
        return self.waitForFlagSafe(interrupt_flag, timeout_s, interrupt)

    def clearInterruptStatus(self):
        interrupt = SingleRegister()
        return self.getInterruptStatus(interrupt)

    def waitForFlagSafe(self, interrupt_flag, timeout_s, interrupt=None):
        """Performs waitForFlag in polling mode.

        Args:
            interrupt_flag: Flag of interrupt type to check the interrupt register against.
            timeout_s: duration to wait for the interrupt in seconds.
            interrupt (optional): Container for the interrupt status register data.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        if interrupt is None:
            interrupt = SingleRegister()
        start = time()
        while (time() - start) < timeout_s:
            status = self.getInterruptStatus(interrupt)
            if (interrupt[0] & interrupt_flag) and status == POZYX_SUCCESS:
                return True
            sleep(PozyxConstants.DELAY_POLLING_FAST)
        return False

    ## \addtogroup core
    # @{

    def setWrite(self, address, data, remote_id=None, local_delay=PozyxConstants.DELAY_LOCAL_WRITE,
                 remote_delay=PozyxConstants.DELAY_REMOTE_WRITE):
        """Writes data to Pozyx registers either locally or remotely.

        Args:
            address: The register address
            data: A ByteStructure - derived object that contains the data to be written.
            remote_id (optional): Remote ID for remote read.
            local_delay (optional): Delay after a local write
            remote_delay (optional): Delay after a remote write

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT

        Examples:
            >>> leds = SingleRegister(0xFF)
            >>> self.setWrite(PozyxRegisters.LED_CONTROL, leds)
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
        """Reads Pozyx register data either locally or remotely.

        Args:
            address: The register address
            data: A ByteStructure - derived object that is the container of the read data.

        Kwargs:
            remote_id: Remote ID for remote read.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT

        Example:
            >>> who_am_i = SingleRegister()
            >>> self.getRead(PozyxRegisters.WHO_AM_I, who_am_i)
            >>> print(who_am_i)
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
        """Activates a Pozyx register function either locally or remotely.

        Args:
            address: The function address

        Kwargs:
            params: A ByteStructure - derived object that contains the parameters for the function.
            data: A ByteStructure - derived object that is the container of the read data.
            remote_id: Remote ID for remote read.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT

        Example:
            >>> self.useFunction(PozyxRegisters.CLEAR_DEVICES)
        """
        if not is_functioncall(function):
            if not self.suppress_warnings:
                warn("Register 0x%0.02x isn't a function register" % function, stacklevel=3)
        params = Data([]) if params is None else params
        data = Data([]) if data is None else data
        if remote_id is None:
            return self.regFunction(function, params, data)
        else:
            return self.remoteRegFunction(remote_id, function, params, data)

    # wait for flag functions

    def checkForFlag(self, interrupt_flag, timeout_s, interrupt=None):
        """Performs waitForFlag_safe and checks against errors or timeouts.

        This abstracts the waitForFlag status check routine commonly encountered
        in more complex library functions and checks the given flag against
        the error flag.

        Args:
            interrupt_flag: Flag of interrupt type to check the interrupt register against.
            timeout_s: duration to wait for the interrupt in seconds
            interrupt (optional): Container for the interrupt status register data.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        if interrupt is None:
            interrupt = SingleRegister()
        error_interrupt_mask = PozyxBitmasks.INT_MASK_ERR
        if self.waitForFlagSafe(interrupt_flag | error_interrupt_mask, timeout_s, interrupt):
            if (interrupt[0] & error_interrupt_mask) == error_interrupt_mask:
                return POZYX_FAILURE
            else:
                return POZYX_SUCCESS
        else:
            return POZYX_TIMEOUT

    ## @}

    ## \addtogroup communication_functions
    # @{

    def readRXBufferData(self, data, offset=0):
        """Reads the device's receive buffer's data completely.

        Args:
            data: Container for the data to be read from the receiver buffer.
            offset (optional): Offset of where in the RX buffer to start read

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE
        """
        if data.byte_size + offset > PozyxConstants.MAX_BUF_SIZE:
            return POZYX_FAILURE

        _MAX_SERIAL_SIZE = PozyxConstants.MAX_SERIAL_SIZE

        status = POZYX_SUCCESS
        runs = int(data.byte_size / _MAX_SERIAL_SIZE)
        total_byte_data = ""
        for i in range(runs):
            partial_data = Data([0] * _MAX_SERIAL_SIZE)
            status &= self.regFunction(PozyxRegisters.READ_RX_DATA, Data([offset + i * _MAX_SERIAL_SIZE,
                                                            partial_data.byte_size]), partial_data)
            total_byte_data += partial_data.byte_data
        partial_data = Data([0] * (data.byte_size - runs * _MAX_SERIAL_SIZE))
        status &= self.regFunction(PozyxRegisters.READ_RX_DATA, Data([offset + runs * _MAX_SERIAL_SIZE,
                                                        partial_data.byte_size]), partial_data)
        total_byte_data += partial_data.byte_data
        data.load_bytes(total_byte_data)
        return status

    # TODO: here it's max serial size but this should be set in PozyxSerial and used as a max size here.
    def writeTXBufferData(self, data, offset=0):
        """Writes data to the device's transmit buffer at the offset address.

        Args:
            data: Data to write to the Pozyx buffer. Has to be a ByteStructure derived object.
            offset (optional): Offset in buffer to start writing data

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE
        """
        if not dataCheck(data):
            data = Data(data)
        if offset + data.byte_size > PozyxConstants.MAX_BUF_SIZE:
            return POZYX_FAILURE

        # have to account for the parameter taking up a byte
        _MAX_SERIAL_SIZE = PozyxConstants.MAX_SERIAL_SIZE - 1

        status = POZYX_SUCCESS
        data = Data(data.transform_to_bytes())
        runs = int(data.byte_size / _MAX_SERIAL_SIZE)
        for i in range(runs):
            status &= self.regFunction(PozyxRegisters.WRITE_TX_DATA, Data([offset + i * _MAX_SERIAL_SIZE]
                                       + data[i * _MAX_SERIAL_SIZE: (i + 1) * _MAX_SERIAL_SIZE]), Data([]))
        return status & self.regFunction(PozyxRegisters.WRITE_TX_DATA, Data([offset + runs * _MAX_SERIAL_SIZE]
                                                             + data[runs * _MAX_SERIAL_SIZE:]), Data([]))

    def sendTXBufferData(self, destination):
        """Sends the transmit buffer's data to the destination device.

        Args:
            destination: Network ID of destination. integer ID or NetworkID(ID)

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE
        """
        return self.sendTX(destination, PozyxConstants.REMOTE_DATA)

    def sendTXRead(self, destination):
        """Sends the read operation's data in the transmit buffer to the destination device.

        Args:
            destination: Network ID of destination. integer ID or NetworkID(ID)

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE
        """
        return self.sendTX(destination, PozyxConstants.REMOTE_READ)

    def sendTXWrite(self, destination):
        """Sends the write operation's data in the transmit buffer to the destination device.

        Args:
            destination: Network ID of destination. integer ID or NetworkID(ID)

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE
        """
        return self.sendTX(destination, PozyxConstants.REMOTE_WRITE)

    def sendTXFunction(self, destination):
        """Sends the function parameters in the transmit buffer to the destination device.

        Args:
            destination: Network ID of destination. integer ID or NetworkID(ID)

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE
        """
        return self.sendTX(destination, PozyxConstants.REMOTE_FUNCTION)

    # Not a TODO but see if these can be chained to make huge and slow DIY chains by adding remote_id
    # make wrapper for this remote remote communication?
    def sendTX(self, destination, operation):
        """Sends the data in the transmit buffer to destination ID. Helper for sendData.

        Args:
            destination: Network ID of destination. integer ID or NetworkID(ID)
            operation: Type of TX operation. These vary depending on the desired operation.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE
        """
        if dataCheck(destination):
            destination = destination[0]
        return self.regFunction(PozyxRegisters.SEND_TX_DATA, TXInfo(destination, operation), Data([]))

    def sendData(self, destination, data):
        """Stores the data in the transmit buffer and then sends it to the device with ID destination.

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

    def getInterruptStatus(self, interrupts, remote_id=None):
        """Obtains the Pozyx's interrupt register.

        Args:
            interrupts: Container for the read data. SingleRegister or Data([0]).
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        return self.getRead(PozyxRegisters.INTERRUPT_STATUS, interrupts, remote_id)

    def getRxInfo(self, rx_info, remote_id=None):
        """Obtain's information on the data the Pozyx received over UWB

        Args:
            rx_info: Container for the RX Info, Data or RXInfo
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        return self.getRead(PozyxRegisters.RX_NETWORK_ID, rx_info, remote_id)

## @}


class PozyxException(IOError):
    """Base class for Pozyx related exceptions"""
    pass


class PozyxConnectionError(PozyxException):
    """Bad connection to Pozyx gives an exception"""
    pass