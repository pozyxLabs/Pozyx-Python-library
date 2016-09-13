import struct

from pypozyx.structures.byte_structure import ByteStructure


def is_reg_readable(reg):
    if (reg >= 0x00 and reg < 0x07) or (reg >= 0x10 and reg < 0x12) or (reg >= 0x15 and reg < 0x21) or (reg >= 0x22 and reg < 0x24) or (reg >= 0x27 and reg < 0x2B) or (reg >= 0x30 and reg < 0x48) or (reg >= 0x50 and reg < 0x89):
        return True
    return False


def is_reg_writable(reg):
    if (reg >= 0x10 and reg < 0x12) or (reg >= 0x15 and reg < 0x21) or (reg >= 0x22 and reg < 0x24) or (reg >= 0x27 and reg < 0x2B) or (reg >= 0x30 and reg < 0x3C) or (reg >= 0x85 and reg < 0x89):
        return True
    return False


def is_functioncall(reg):
    if (reg >= 0xB0 and reg < 0xBC) or (reg >= 0xC0 and reg < 0xC9):
        return True
    return False


def dataCheck(data):
    if not(Data in type(data).__bases__ or ByteStructure in type(data).__bases__ or Data is type(data) or XYZ in type(data).__bases__):
        return False
    return True


class XYZ(ByteStructure):
    physical_convert = 1

    byte_size = 12
    data_format = 'iii'

    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z
        self.data = [x, y, z]

    def load(self, data=[0] * 3, convert=1):
        self.data = data
        if convert:
            self.x = data[0] / self.physical_convert
            self.y = data[1] / self.physical_convert
            self.z = data[2] / self.physical_convert
        else:
            self.x = data[0]
            self.y = data[1]
            self.z = data[2]

    def update_data(self):
        try:
            if self.data != [self.x, self.y, self.z]:
                self.data = [self.x, self.y, self.z]
        except:
            return

    def __str__(self):
        return 'X: {self.x}, Y: {self.y}, Z: {self.z}'.format(self=self)


class Data(ByteStructure):

    def __init__(self, data, data_format=None):
        self.data = data
        if data_format == None:
            data_format = 'B' * len(data)
        self.data_format = data_format
        self.set_packed_size()

# definitely unused in favor of Data


class UniformData(ByteStructure):

    def __init__(self, data=[], format_type='b', data_size=1):
        self.data = data
        self.format_type = format_type
        self.data_size = data_size
        self.data_length = len(data)
        self.byte_size = data_size * data_length
        self.data_format = format_type * data_length


class SingleRegister(Data):

    def __init__(self, value=0, size=1, signed=1):
        if size == 1:
            data_format = 'b'
        elif size == 2:
            data_format = 'h'
        elif size == 4:
            data_format = 'i'
        if signed == 1:
            data_format = data_format.capitalize()
        Data.__init__(self, [value], data_format)
