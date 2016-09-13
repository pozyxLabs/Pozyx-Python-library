import struct

# TODO: Definitely add more 'WHY' to this entire class, document it well
# and wisely.


class ByteStructure():
    byte_size = 4
    data_format = 'BBBB'

    #
    def __init__(self):
        self.byte_data = '00' * self.byte_size
        #self.data = []
        self.bytes_to_data()

    #
    def load_bytes(self, byte_data):
        # bytes is hexadecimal array or just a string entirely. String entirely
        # may be easiest.
        self.byte_data = byte_data
        self.bytes_to_data()

    #
    def bytes_to_data(self):
        s = ''.encode()
        for i in range(int(len(self.byte_data) / 2)):
            index = 2 * i
            s += struct.pack('B',
                             int(self.byte_data[index:index + 2], 16))
        self.load_packed(s)

    #
    def load_packed(self, s):
        index = 0
        self.data = [0] * len(self.data_format)
        for i in range(len(self.data_format)):
            data_len = struct.calcsize(self.data_format[i])
            self.data[i] = struct.unpack(self.data_format[i], s[
                                         index:index + data_len])[0]
            index += data_len
        self.load(self.data)

    #
    def load_hex_string(self):
        byte_data = self.transform_to_bytes()
        s = ''
        for i in range(len(byte_data)):
            s += '%0.2x' % byte_data[i]
        self.byte_data = s

    #
    def transform_to_bytes(self):
        new_format = ''
        for i in range(len(self.data)):
            new_format += 'B' * struct.calcsize(self.data_format[i])
        return self.transform_data(new_format)

    #
    def set_packed_size(self):
        new_format = ''
        for i in range(len(self.data)):
            new_format += 'B' * struct.calcsize(self.data_format[i])
        self.byte_size = 1 * len(new_format)

    #
    def set_unpacked_size(self):
        self.byte_size = struct.calcsize(self.data_format)

    #
    def transform_data(self, new_format):
        s = ''.encode()
        for i in range(len(self.data)):
            s += struct.pack(self.data_format[i], self.data[i])
        return list(struct.unpack(new_format, s))

    #
    def change_data(self, index, new_data):
        if type(new_data) == int:
            self.data[index] = new_data
        elif type(new_data) == list:
            for i in range(len(new_data)):
                self.data[index + i] == new_data[i]
        else:
            print("Trying to change data with invalid new values (use int or list)")

    #
    def load(self, data, convert=1):
        pass

    #
    def update_data(self):
        pass

    # Python Data Object functionality.
    def __getitem__(self, key):
        return self.data[key]

    def __setitem__(self, key, value):
        self.data[key] = value

    def __len__(self):
        return len(self.data)

    def __eq__(self, other):
        if len(self) != len(other):
            return False
        for i in range(len(self)):
            if self[i] != other[i]:
                return False
        return True

    def __ne__(self, other):
        return self.data != other.data

    def __str__(self):
        s = ''
        for i in range(len(self.data)):
            if i > 0:
                s += ', '
            s += str(self.data[i])
        return s

    # should always be tailored to specific data structure.
    def __setattr__(self, name, value):
        # keep the regular assignment in place
        self.__dict__[name] = value
        # keep the data up to date when setting a variable.
        self.update_data()
