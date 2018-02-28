#!/usr/bin/env python
""" pypozyx.structures.byte_structure - contains the ByteStructure class, thank you struct."""

import struct


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
        self.byte_data = '00' * self.byte_size
        self.bytes_to_data()

    def load_bytes(self, byte_data):
        """Loads a hex byte array in the structure's data"""
        self.byte_data = byte_data
        self.bytes_to_data()

    def bytes_to_data(self):
        """Transforms hex data into packed UINT8 byte values"""
        s = ''.encode()
        for i in range(int(len(self.byte_data) / 2)):
            index = 2 * i
            s += struct.pack('B',
                             int(self.byte_data[index:index + 2], 16))
        self.load_packed(s)

    def load_packed(self, packed):
        """Unpacks the packed UINT8 bytes in their right format as given in data_format"""
        index = 0
        self.data = [0] * len(self.data_format)
        for i in range(len(self.data_format)):
            data_len = struct.calcsize(self.data_format[i])
            self.data[i] = struct.unpack(self.data_format[i], packed[
                                         index:index + data_len])[0]
            index += data_len
        self.load(self.data)

    def load_hex_string(self):
        """Loads the data's hex string for sending"""
        byte_data = self.transform_to_bytes()
        s = ''
        for i in range(len(byte_data)):
            s += '%0.2x' % byte_data[i]
        self.byte_data = s

    def transform_to_bytes(self):
        """Transforms the data to a UINT8 bytestring in hex"""
        new_format = ''
        for i in range(len(self.data)):
            new_format += 'B' * struct.calcsize(self.data_format[i])
        return self.transform_data(new_format)

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
        s = ''.encode()
        for i in range(len(self.data)):
            s += struct.pack(self.data_format[i], self.data[i])
        return list(struct.unpack(new_format, s))

    def change_data(self, index, new_data):
        """Changes the internal contained data of the structure. Good for large structures"""
        if type(new_data) == int:
            self.data[index] = new_data
        elif type(new_data) == list:
            for i in range(len(new_data)):
                self.data[index + i] == new_data[i]
        else:
            print("Trying to change data with invalid new values (use int or list)")

    # TODO make convert use True/False not 1/0
    def load(self, data, convert=True):
        """Loads data in its relevant class components."""
        raise NotImplementedError(
            'load(data) should be customised for every derived structure')

    def update_data(self):
        """Updates the class's data when one of its components has changed."""
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
        self.update_data()
