#!/usr/bin/env python
# TODO move this in the RST files.
"""
pypozyx.structures.generic - introduces generic data structures derived from ByteStructure

Generic Structures

As the name implies, contains generic structures whose specific use is up to the
user. You should use SingleRegister where applicable when reading/writing
a single register, and use Data for larger data structures.

Structures contained:
Data
    THE generic data structure, a powerful way of constructing arbitrarily
    formed packed data structures
XYZ
    A generic XYZ data structure that is used in much 3D sensor data
SingleRegister
    Data resembling a single register. Can choose size and whether signed.
UniformData
    A variation on Data with all data being a uniform format. Questionably useful.

The use of Data:
Data creates a packed data structure with size and format that is entirely the user's choice.
The format follows the one used in struct, where b is a byte, h is a 2-byte int, and
i is a default-sized integer, and f is a float. In capitals, these are signed.
So, to create a custom construct consisting of 4 uint16 and a single int, the
following code can be used.

  >>> d = Data([0] * 5, 'HHHHi')

or

  >>> data_format = 'HHHHi'
  >>> d = Data([0] * len(data_format), data_format)
"""

from pypozyx.structures.byte_structure import ByteStructure


def is_reg_readable(reg):
    """Returns whether a Pozyx register is readable."""
    if (0x00 <= reg < 0x07) or (0x10 <= reg < 0x12) or (0x14 <= reg < 0x22) or (0x22 <= reg < 0x24) or (
            0x27 <= reg < 0x2B) or (0x30 <= reg < 0x48) or (0x4E <= reg < 0x89):
        return True
    return False


def is_reg_writable(reg):
    """Returns whether a Pozyx register is writeable."""
    if (0x10 <= reg < 0x12) or (0x14 <= reg < 0x22) or (0x22 <= reg < 0x24) or (0x27 <= reg < 0x2B) or (
            0x30 <= reg < 0x3C) or (0x85 <= reg < 0x89):
        return True
    return False


def is_functioncall(reg):
    """Returns whether a Pozyx register is a Pozyx function."""
    if (0xB0 <= reg < 0xBC) or (0xC0 <= reg < 0xC9):
        return True
    return False


def dataCheck(data):
    """Returns whether an object is part of the ByteStructure-derived classes or not.

    The function checks the base classes of the passed data object. This function enables
    many library functions to be passed along its data as either an int/list or the properly
    intended data structure. For example, the following code will result in the
    same behaviour::

      >>> p.setCoordinates([0, 0, 0])
      >>> # or
      >>> coords =  Coordinates()
      >>> p.setCoordinates(coords)

    AND

      >>> p.setNetworkId(0x6000)
      >>> # or
      >>> n = NetworkID(0x6000)
      >>> p.setNetworkId(n)

    Note that this only works for functions where you change one of the Pozyx's
    settings. When reading data from the Pozyx, you have to pass along the correct
    data structure.

    Using dataCheck:
    You might want to use this in your own function, as it makes it more robust
    to whether an int or list gets sent as a parameter to your function, or a
    ByteStructure-like object. If so, you can perform::

      >>> if not dataCheck(sample): # assume a is an int but you want it to be a SingleRegister
      >>>     sample = SingleRegister(sample)

    """
    if not(Data in type(data).__bases__ or ByteStructure in type(data).__bases__ or Data is type(data) or XYZ in type(data).__bases__):
        return False
    return True


class XYZ(ByteStructure):
    """
    Generic XYZ data structure consisting of 3 integers x, y, and z.

    Not recommended to use in practice, as relevant sensor data classes are derived from this.

    If deriving this, don't forget to implement your own update_data function, or data will
    be [x, y, z] consistently instead of [..., x, y, z, ...].
    """
    physical_convert = 1

    byte_size = 12
    data_format = 'iii'

    def __init__(self, x=0, y=0, z=0):
        """Initializes the XYZ or XYZ-derived object."""
        self.x = x
        self.y = y
        self.z = z
        self.data = [x, y, z]

    def load(self, data, convert=True):
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
    """Data allows the user to define arbitrary data structures to use with Pozyx.

    The Leatherman of ByteStructure-derived classes, Data allows you to create your own
    library-compatible packed data structures. Also for empty data, this is used.

    The use of Data:
    Data creates a packed data structure with size and format that is entirely the user's choice.
    The format follows the one used in struct, where b is a byte, h is a 2-byte int, and
    i is a default-sized integer, and f is a float. In capitals, these are unsigned.
    So, to create a custom construct consisting of 4 uint16 and a single int, the
    following code can be used.

      >>> d = Data([0] * 5, 'HHHHi')

    or

      >>> data_format = 'HHHHi'
      >>> d = Data([0] * len(data_format), data_format)

    Args:
        data (optional): Data contained in the data structure. When no data_format is passed, these are assumed UInt8 values.
        data_format (optional): Custom data format for the data passed.
    """

    def __init__(self, data=None, data_format=None):
        if data is None:
            data = []
        self.data = data
        if data_format is None:
            data_format = 'B' * len(data)
        self.data_format = data_format
        self.set_packed_size()
        self.byte_data = '00' * self.byte_size

    def load(self, data, convert=True):
        self.data = data


class SingleRegister(Data):
    """ SingleRegister is container for the data from a single Pozyx register.

    By default, this represents a UInt8 register. Used for both reading and writing.
    The size and whether the data is a 'signed' integer are both changeable by the
    user using the size and signed keyword arguments.

    Args:
        value (optional): Value of the register.
        size (optional): Size of the register. 1, 2, or 4. Default 1.
        signed (optional): Whether the data is signed. unsigned by default.
        print_hex (optional): How to print the register output. Hex by default. Special options are 'hex' and 'bin'
            other things, such as 'dec', will return decimal output.
    """
    byte_size = 1
    data_format = 'B'

    def __init__(self, value=0, size=1, signed=0, print_style='hex'):
        self.print_style = print_style
        if size == 1:
            data_format = 'b'
        elif size == 2:
            data_format = 'h'
        elif size == 4:
            data_format = 'i'
        else:
            raise ValueError("Size should be 1, 2, or 4")
        if signed == 0:
            data_format = data_format.capitalize()
        Data.__init__(self, [value], data_format)

    def load(self, data, convert=True):
        self.data = data

    @property
    def value(self):
        return self.data[0]

    @value.setter
    def value(self, new_value):
        self.data[0] = new_value

    def __str__(self):
        if self.print_style is 'hex':
            return hex(self.data[0]).capitalize()
        elif self.print_style is 'bin':
            return bin(self.data[0])
        else:
            return str(self.data[0])


class SingleSensorValue(ByteStructure):
    """
    Generic Single Sensor Value data structure.

    Not recommended to use in practice, as relevant sensor data classes are derived from this.

    If deriving this, don't forget to implement your own update_data function, or data will
    be [value] consistently instead of [..., value, ...].
    """
    physical_convert = 1

    byte_size = 4
    data_format = 'i'

    def __init__(self, value=0):
        """Initializes the XYZ or XYZ-derived object."""
        self.value = value
        self.load([value])

    def load(self, data=None, convert=True):
        self.data = [0] if data is None else data

        if convert:
            self.value = float(self.data[0]) / self.physical_convert
        else:
            self.value = self.data[0]

    def update_data(self):
        try:
            if self.data != [self.value]:
                self.data = [self.value]
        except:
            return

    def __str__(self):
        return 'Value: {self.value}'.format(self=self)
