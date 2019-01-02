#!/usr/bin/env python
""" pypozyx.structures.byte_structure - contains the ByteStructure class, thank you struct."""

import struct




class HexString(object):
    def __init__(self, string):
        self._string = string
        self._bytes = [self[i] for i in range(len(self))]

    @property
    def string(self):
        return self._string

    def __getitem__(self, i):
        return self._string[2 * i: 2 * (i + 1)]

    def __len__(self):
        return len(self._string) // 2

    def __iter__(self):
         return iter(self._bytes)


# TODO make data a property
class ByteStructure(object):
    """
    The ByteStructure class is the base class that all custom structs inherit
    their basic functionality from. It implements the low-level functionality
    that makes it easy to make use arbitrary struct formats in the interface
    with Pozyx.
    """
    byte_size = 4
    data_format = 'BBBB'

    def __init__(self):
        """Initializes the structures byte data and data arrays"""
        self.byte_data = HexString('00' * self.byte_size)
        self.bytes_to_data()
        self.data = [0] * len(self.data_format)

    def load_bytes(self, byte_data):
        """Loads a hex byte array in the structure's data"""
        self.byte_data = HexString(byte_data)
        self.bytes_to_data()

    def bytes_to_data(self):
        """Transforms hex data into packed UINT8 byte values"""
        s = ''.encode()
        for byte in self.byte_data:
            print(byte)
            s += struct.pack('B', int(byte, 16))
        self.load_packed(s)

    def load_packed(self, packed):
        """Unpacks the packed UINT8 bytes in their right format as given in data_format"""
        index = 0
        data = [0] * len(self.data_format)
        for i in range(len(self.data_format)):
            data_len = struct.calcsize(self.data_format[i])
            data[i] = struct.unpack(self.data_format[i], packed[index:index + data_len])[0]
            index += data_len
        self.load(data)
                  
    @property
    def hex_string(self):
        hex_string = ''
        for byte in self.transform_to_bytes():
            hex_string += '%0.2x' % byte
        return hex_string

    @property
    def bytes_format(self):
        return 'B' * sum([struct.calcsize(data_format_part) for data_format_part in self.data_format])
    
    @property
    def bytes_data(self):
        return 

    def load_hex_string(self):
        """Loads the data's hex string for sending"""
        self.byte_data = self.hex_string

    def transform_to_bytes(self):
        """Transforms the data to a UINT8 bytestring in hex"""
        return self.transform_data(self.bytes_format)

    def set_packed_size(self):
        """Sets the size (bytesize) to the structures data format, but packed"""
        new_format = ''
        for i in range(len(self.data)):
            new_format += 'B' * struct.calcsize(self.data_format[i])
        self.byte_size = 1 * len(new_format)

    def set_unpacked_size(self):
        """Sets the size (bytesize) to the structures data format, unpacked"""
        self.byte_size = struct.calcsize(self.data_format)

    def transform_data(self, new_format):
        """Transforms the data to a new format, handy for decoding to bytes"""
        s = bytes()
        for data_part_format, data_part_value in zip(self.data_format, self.data):
            s += struct.pack(data_part_format, data_part_value)
        return list(struct.unpack(new_format, s))

    def load(self, data, convert=True):
        """Loads data in its relevant class components."""
        self.data = data
        # raise NotImplementedError(
        #     'load(data) should be customised for every derived structure')

    def update_data(self):
        """Updates the class's data when one of its components has changed."""
        pass

    def update_values(self):
        """Updates the class's members when its data has changed."""
        pass

    def __getitem__(self, key):
        return self.data[key]

    def __setitem__(self, key, value):
        self.data[key] = value

    def __len__(self):
        """Returns the size of the structure's data"""
        return len(self.data)

    def __eq__(self, other):
        """Returns whether the structures contain the same data"""
        return self.data == other.data and self.data_format == other.data_format

    def __str__(self):
        """Returns a string that should be tailored to each ByteStructure-derived class."""
        s = ''
        for i in range(len(self.data)):
            if i > 0:
                s += ', '
            s += str(self.data[i])
        return s

    def __setattr__(self, name, value):
        """Made to link the direct data updates to also update the data array"""
        self.__dict__[name] = value
        if name != "data":
            self.update_data()
        else:
            # NOTE: does not work with individual access of data
            self.update_values()


if __name__ == '__main__':
    byte_structure = ByteStructure()
    byte_structure.load([1, 2, 3, 4])
    assert byte_structure.hex_string == '01020304'
    from pypozyx import Coordinates
    coordinates = Coordinates(1, 2999, 3)
    print(coordinates.hex_string)
