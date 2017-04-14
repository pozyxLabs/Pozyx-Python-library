#!/usr/bin/env python
"""
pypozyx.structures.device - contains various classes representing device data

Structures contained
--------------------
DeviceCoordinates
    consists of a device's ID, flag, and coordinates
DeviceRange
    consists of a range measurements timestamp, distance, and RSS
NetworkID
    container for a device's ID. Prints in 0xID format.
DeviceList
    container for a list of IDs. Can be initialized through size and/or IDs.
UWBSettings
    contains all of the UWB settings: channel, bitrate, prf, plen, and gain.
"""

from pypozyx.definitions.constants import *
from pypozyx.structures.byte_structure import ByteStructure
from pypozyx.structures.generic import Data
from pypozyx.structures.sensor_data import Coordinates


class DeviceCoordinates(ByteStructure):
    """
    Container for both reading and writing device coordinates from and to Pozyx.

    The keyword arguments are at once its properties.

    Kwargs:
        network_id: Network ID of the device
        flag: Type of the device. Tag or anchor.
        pos: Coordinates of the device. Coordinates().
    """
    byte_size = 15
    data_format = 'HBiii'

    def __init__(self, network_id=0, flag=0, pos=Coordinates()):
        """
        Initializes the DeviceCoordinates object.

        Kwargs:
            network_id: Network ID of the device
            flag: Type of the device. Tag or anchor.
            pos: Coordinates of the device. Coordinates().
        """
        self.network_id = network_id
        self.flag = flag
        self.pos = pos
        self.data = [network_id, flag, int(pos.x), int(pos.y), int(pos.z)]

    def load(self, data):
        self.data = data
        self.network_id = data[0]
        self.flag = data[1]
        self.pos = Coordinates(data[2], data[3], data[4])

    def update_data(self):
        try:
            if self.data != [self.network_id, self.flag,
                             self.pos.x, self.pos.y, self.pos.z]:
                self.data = [self.network_id, self.flag,
                             self.pos.x, self.pos.y, self.pos.z]
        except:
            return

    def __str__(self):
        return "ID: 0x{self.network_id:x}, flag: {self.flag}, ".format(self=self) + str(self.pos)


class DeviceRange(ByteStructure):
    """
    Container for the device range data, resulting from a range measurement.

    The keyword arguments are at once its properties.

    Kwargs:
        timestamp: Timestamp of the range measurement
        distance: Distance measured by the device.
        RSS: Signal strength during the ranging measurement.
    """
    byte_size = 10
    data_format = 'IIh'

    def __init__(self, timestamp=0, distance=0, RSS=0):
        """Initializes the DeviceRange object."""
        self.timestamp = timestamp
        self.distance = distance
        self.RSS = RSS
        self.data = [timestamp, distance, RSS]

    def load(self, data):
        self.data = data
        self.timestamp = data[0]
        self.distance = data[1]
        self.RSS = data[2]

    def update_data(self):
        try:
            if self.data != [self.timestamp, self.distance, self.RSS]:
                self.data = [self.timestamp, self.distance, self.RSS]
        except:
            return

    def __str__(self):
        return '{self.timestamp}ms, {self.distance}mm, {self.RSS}dBm'.format(self=self)


class NetworkID(Data):
    """
    Container for a device's network ID.

    Kwargs:
        network_id: The network ID of the device.
    """

    def __init__(self, network_id=0):
        """Initializes the NetworkID object."""
        Data.__init__(self, [network_id], 'H')
        self.id = network_id

    def load(self, data):
        self.data = data
        self.id = data[0]

    def update_data(self):
        try:
            if self.data != [self.id]:
                self.data = [self.id]
        except:
            return

    def __str__(self):
        return "0x%0.4x" % self.id


class DeviceList(Data):
    """
    Container for a list of device IDs.

    Using list_size is recommended when having just used getDeviceListSize, while ids
    is recommended when one knows the IDs. When using one, the other automatically
    gets its respective value. Therefore, only use on of both.

    Note also that DeviceList(list_size=1) is the same as NetworkID().

    Kwargs:
        ids: Array of known or unknown device IDs. Empty by default.
        list_size: Size of the device list.
    """

    def __init__(self, ids=[], list_size=0):
        """Initializes the DeviceList object with either IDs or its size."""
        if list_size != 0 and ids == []:
            Data.__init__(self, [0] * list_size, 'H' * list_size)
        else:
            Data.__init__(self, ids, 'H' * len(ids))

    def __str__(self):
        s = 'IDs: '
        for i in range(len(self)):
            if i > 0:
                s += ', '
            s += '0x%0.4x' % self[i]
        return s

    def load(self, data):
        for i in range(len(data)):
            self.data[i] = data[i]


class UWBSettings(ByteStructure):
    """
    Container for a device's UWB settings.

    Its keyword arguments are at once its properties.

    It also provides parsing functions for all its respective properties,
    which means this doesn't need to be done by users. These functions are
    parse_prf, parse_plen and parse_bitrate.

    You can also directly print the UWB settings, resulting in the following
    example output:
    "CH: 1, bitrate: 850kbit/s, prf: 16MHz, plen: 1024 symbols, gain: 15.0dB"

    Kwargs:
        channel: UWB channel of the device. See POZYX_UWB_CHANNEL.
        bitrate: Bitrate of the UWB commmunication. See POZYX_UWB_RATES.
        prf: Pulse repeat frequency of the UWB. See POZYX_UWB_RATES.
        plen: Preamble length of the UWB packets. See POZYX_UWB_PLEN.
        gain_db: Gain of the UWB transceiver, a float value. See POZYX_UWB_GAIN.
    """
    byte_size = 7
    data_format = 'BBBBf'

    def __init__(self, channel=0, bitrate=0, prf=0, plen=0, gain_db=0.0):
        """Initializes the UWB settings."""
        self.channel = channel
        self.bitrate = bitrate
        self.prf = prf
        self.plen = plen
        self.gain_db = float(gain_db)
        self.data = [self.channel, self.bitrate,
                     self.prf, self.plen, self.gain_db]

    def load(self, data):
        self.channel = data[0]
        self.bitrate = data[1] & 0x3F
        self.prf = (data[1] & 0xC0) >> 6
        self.plen = data[2]
        self.gain_db = float(data[3]) / 2
        self.data = [self.channel, self.bitrate,
                     self.prf, self.plen, self.gain_db]

    def update_data(self):
        try:
            if self.data != [self.channel, self.bitrate,
                             self.prf, self.plen, self.gain_db]:
                self.data = [self.channel, self.bitrate,
                             self.prf, self.plen, self.gain_db]
        except:
            return

    def parse_bitrate(self):
        """Parses the bitrate to be humanly readable."""
        bitrates = {0: '110kbit/s', 1: '850kbit/s', 2: '6.81Mbit/s'}
        try:
            return bitrates[self.bitrate]
        except:
            return 'invalid bitrate'

    def parse_prf(self):
        """Parses the pulse repetition frequency to be humanly readable."""
        prfs = {1: '16 MHz', 2: '64 MHz'}
        try:
            return prfs[self.prf]
        except:
            return 'invalid pulse repetitions frequency (PRF)'

    def parse_plen(self):
        """Parses the preamble length to be humanly readable."""
        plens = {0x0C: '4096 symbols', 0x28: '2048 symbols', 0x18: '1536 symbols', 0x08: '1024 symbols',
                 0x34: '512 symbols', 0x24: '256 symbols', 0x14: '128 symbols', 0x04: '64 symbols'}
        try:
            return plens[self.plen]
        except:
            return 'invalid preamble length'

    def __str__(self):
        return "CH: {}, bitrate: {}, prf: {}, plen: {}, gain: {}dB".format(self.channel, self.parse_bitrate(), self.parse_prf(), self.parse_plen(), self.gain_db)
