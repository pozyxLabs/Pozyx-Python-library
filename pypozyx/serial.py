from pypozyx.definitions.constants import *
from pypozyx.definitions.registers import *

from pypozyx.lib import PozyxLib

from pypozyx.structures.generic import Data, SingleRegister

import serial
import time


class PozyxSerial(PozyxLib):

    def __enter__(self):
        return self

    def __exit__(self,exc_type,exc_value,traceback):
        count = 0
        while (self.ser.is_open and count < 100):
            self.ser.close()
            count += 1
        if self.ser.is_open:
            print("ERROR: Serial port failed to close")
            return POZYX_FAILURE

    def __init__(self, port, baudrate=115200, timeout=0.2, mode=MODE_POLLING, print_output=False):
        self.print_output = print_output
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self._mode = mode

        time.sleep(0.25)

        regs = Data([0, 0, 0])
        if self.regRead(POZYX_WHO_AM_I, regs) == POZYX_FAILURE:
            return POZYX_FAILURE

        self._hw_version = regs[1]
        self._sw_version = regs[2]

        if regs[0] != 0x43:
            return POZYX_FAILURE

    def regWrite(self, address, data):
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
        params.load_hex_string()
        s = 'F,%0.2x,%s,%i\r' % (address, params.byte_data, data.byte_size + 1)
        try:
            r = self.serialExchange(s)
        except:
            return POZYX_FAILURE
        if len(data) > 0:
            data.load_bytes(r[2:])
        return int(r[0:2], 16)

    def waitForFlag(self, interrupt_flag, timeout_s, interrupt=SingleRegister()):
        return self.waitForFlag_safe(interrupt_flag, timeout_s, interrupt)

    def waitForFlag_safe(self, interrupt_flag, timeout_s, interrupt=SingleRegister()):
        start = time.time()
        while(time.time() - start < timeout_s):
            time.sleep(POZYX_DELAY_POLLING)
            status = self.regRead(POZYX_INT_STATUS, interrupt)
            if (interrupt[0] & interrupt_flag) and status == POZYX_SUCCESS:
                return True
        return False
