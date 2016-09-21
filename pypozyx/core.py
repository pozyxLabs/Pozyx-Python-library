from pypozyx.definitions.registers import POZYX_TX_DATA, POZYX_TX_SEND, POZYX_RX_DATA, POZYX_INT_STATUS, POZYX_RX_NETWORK_ID
from pypozyx.definitions.constants import POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT, MAX_BUF_SIZE, MAX_SERIAL_SIZE
from pypozyx.definitions.bitmasks import POZYX_INT_STATUS_ERR, POZYX_INT_STATUS_FUNC

from pypozyx.structures.generic import Data, is_functioncall, is_reg_readable, is_reg_writable, SingleRegister


class PozyxCore():

    def __init__(self):
        pass

    def regRead(self, address, data):
        pass

    def regWrite(self, address, data):
        pass

    def regFunction(self, address, params, data):
        pass

    def waitForFlag(self, interrupt_flag, timeout_ms, interrupt=None):
        pass

    def waitForFlag_safe(self, interrupt_flag, timeout_ms, interrupt=None):
        pass

    def checkForFlag(self, interrupt_flag, timeout_ms, interrupt=SingleRegister()):
        if self.waitForFlag_safe(interrupt_flag | POZYX_INT_STATUS_ERR, timeout_ms, interrupt):
            if (interrupt[0] & POZYX_INT_STATUS_ERR) == POZYX_INT_STATUS_ERR:
                return POZYX_FAILURE
            else:
                return POZYX_SUCCESS
        else:
            return POZYX_TIMEOUT

    def remoteRegWrite(self, destination, address, data):
        if is_reg_writable(address) == 0:
            return POZYX_FAILURE
        if len(data) > MAX_BUF_SIZE - 1:
            return POZYX_FAILURE

        # Alternative way:
        # send_data = Data([address] + data.data, 'B' + data.data_format)
        # status = self.writeTXBufferData(send_data)
        send_data = Data([0, address] + data.data, 'BB' + data.data_format)
        status = self.regFunction(POZYX_TX_DATA, send_data, Data([]))
        if status == POZYX_FAILURE:
            return status

        self.regRead(POZYX_INT_STATUS, SingleRegister())
        params = Data([destination, 0x04], 'HB')
        status = self.regFunction(POZYX_TX_SEND, params, Data([]))
        if status == POZYX_FAILURE:
            return status
        return self.checkForFlag(POZYX_INT_STATUS_FUNC, 0.1)

    def remoteRegRead(self, destination, address, data):
        if is_reg_readable(address) == 0:
            return POZYX_FAILURE
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
        if is_functioncall(address) == 0:
            return POZYX_FAILURE

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

    def readRXBufferData(self, data):
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
        if offset + data.byte_size > MAX_BUF_SIZE:
            return POZYX_FAILURE

        status = POZYX_SUCCESS
        data = Data(data.transform_to_bytes())
        runs = int(data.byte_size / MAX_SERIAL_SIZE)
        for i in range(runs):
            params = Data([i * MAX_SERIAL_SIZE] + data[i *
                                                       MAX_SERIAL_SIZE: (i + 1) * MAX_SERIAL_SIZE])
            status &= self.regFunction(POZYX_TX_DATA, params, Data([]))
        params = Data([runs * MAX_SERIAL_SIZE] +
                      data[runs * MAX_SERIAL_SIZE:])
        return status & self.regFunction(POZYX_TX_DATA, params, Data([]))

    def sendTXBufferData(self, destination):
        params = Data([destination, 0x06], 'HB')
        status = self.regFunction(POZYX_TX_SEND, params, [])
        return status

    def sendData(self, destination, data):
        self.writeTXBufferData(data)
        self.sendTXBufferData(destination)
