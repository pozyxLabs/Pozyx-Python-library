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

from pypozyx.definitions.constants import PozyxConstants
from pypozyx.structures.byte_structure import ByteStructure
from pypozyx.structures.generic import Data, SingleRegister
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
        self.data = [network_id, flag, int(pos.x), int(pos.y), int(pos.z)]

    def load(self, data):
        self.data = data

    def __str__(self):
        return "ID: 0x{:04X}, flag: {}, ".format(self.network_id, self.flag) + str(self.pos)

    @property
    def network_id(self):
        return self.data[0]

    @network_id.setter
    def network_id(self, value):
        self.data[0] = value

    @property
    def flag(self):
        return self.data[1]

    @flag.setter
    def flag(self, value):
        self.data[1] = value

    @property
    def pos(self):
        return Coordinates(self.data[2], self.data[3], self.data[4])

    @pos.setter
    def pos(self, value):
        self.data[2] = value.x
        self.data[3] = value.y
        self.data[4] = value.z


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

    # TODO should ideally be rss not RSS
    def __init__(self, timestamp=0, distance=0, RSS=0):
        """Initializes the DeviceRange object."""
        self.data = [timestamp, distance, RSS]

    def load(self, data):
        self.data = data

    def __str__(self):
        return '{self.timestamp} ms, {self.distance} mm, {self.RSS} dBm'.format(self=self)

    @property
    def timestamp(self):
        return self.data[0]

    @timestamp.setter
    def timestamp(self, value):
        self.data[0] = value

    @property
    def distance(self):
        return self.data[1]

    @distance.setter
    def distance(self, value):
        self.data[1] = value

    @property
    def RSS(self):
        return self.data[2]

    @RSS.setter
    def RSS(self, value):
        self.data[2] = value


def parse_id_str(id_):
    if id_ is None:
        return None
    if isinstance(id_, str):
        if id_.startswith("0x"):
            return str(int(id_, 16))
        else:
            return id_
    elif isinstance(id_, int):
        return str(id_)


def parse_id_int(id_):
    if id_ is None:
        return None
    return int(parse_id_str(id_))


class NetworkID(SingleRegister):
    """
    Container for a device's network ID.

    Kwargs:
        network_id: The network ID of the device.
    """

    def __init__(self, network_id=0):
        """Initializes the NetworkID object."""
        super(NetworkID, self).__init__(parse_id_int(network_id), 2, signed=False)

    def __str__(self):
        return "0x{:04X}".format(self.id)

    @property
    def id(self):
        return self.data[0]

    @id.setter
    def id(self, value):
        self.data[0] = value

    @property
    def network_id(self):
        return self.data[0]

    @network_id.setter
    def network_id(self, value):
        self.data[0] = value


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

    def __init__(self, ids=None, list_size=0):
        """Initializes the DeviceList object with either IDs or its size."""
        ids = [] if ids is None else ids
        if list_size != 0 and ids == []:
            Data.__init__(self, [0] * list_size, 'H' * list_size)
        else:
            Data.__init__(self, ids, 'H' * len(ids))

    def __str__(self):
        return 'IDs: ' + ', '.join([str(id) for id in self])

    def load(self, data, convert=False):
        self.data = data

    @property
    def list_size(self):
        return len(self.data)


class RXInfo(ByteStructure):
    byte_size = 3
    data_format = 'HB'

    def __init__(self, remote_id=0, amount_of_bytes=0):
        """Initialises the RX Info structure"""
        self.data = [remote_id, amount_of_bytes]

    def load(self, data):
        self.data = data

    @property
    def remote_id(self):
        return self.data[0]

    @remote_id.setter
    def remote_id(self, value):
        self.data[0] = value

    @property
    def amount_of_bytes(self):
        return self.data[1]

    @amount_of_bytes.setter
    def amount_of_bytes(self, value):
        self.data[1] = value


class TXInfo(ByteStructure):
    """Container for data transmission meta information

    Args:
        remote_id: ID to transmit to
        operation: remote operation to execute
    """
    byte_size = 3
    data_format = 'HB'

    def __init__(self, remote_id, operation=PozyxConstants.REMOTE_DATA):
        self.data = [remote_id, operation]

    @property
    def remote_id(self):
        return self.data[0]

    @remote_id.setter
    def remote_id(self, value):
        self.data[0] = value

    @property
    def operation(self):
        return self.data[1]

    @operation.setter
    def operation(self, value):
        self.data[1] = value


class UWBMapping:
    BITRATES = {0: '110 kbit/s', 1: '850 kbit/s', 2: '6.81 Mbit/s'}
    PRFS = {1: '16 MHz', 2: '64 MHz'}
    PREAMBLE_LENGTHS = {0x0C: '4096 symbols', 0x28: '2048 symbols', 0x18: '1536 symbols', 0x08: '1024 symbols',
                 0x34: '512 symbols', 0x24: '256 symbols', 0x14: '128 symbols', 0x04: '64 symbols'}

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
        self.data = [channel, bitrate + (prf << 6), plen, int(2 * gain_db)]

    def load(self, data, convert=False):
        self.data = data

    def parse_bitrate(self):
        """Parses the bitrate to be humanly readable."""
        try:
            return UWBMapping.BITRATES[self.bitrate]
        except KeyError:
            return 'invalid bitrate'

    def parse_prf(self):
        """Parses the pulse repetition frequency to be humanly readable."""

        try:
            return UWBMapping.PRFS[self.prf]
        except KeyError:
            return 'invalid pulse repetitions frequency (PRF)'

    def parse_plen(self):
        """Parses the preamble length to be humanly readable."""

        try:
            return UWBMapping.PREAMBLE_LENGTHS[self.plen]
        except KeyError:
            return 'invalid preamble length'

    def __str__(self):
        return "CH: {}, bitrate: {}, prf: {}, plen: {}, gain: {} dB".format(self.channel, self.parse_bitrate(),
                                                                            self.parse_prf(), self.parse_plen(),
                                                                            self.gain_db)

    @property
    def channel(self):
        return self.data[0]

    @channel.setter
    def channel(self, value):
        self.data[0] = value

    @property
    def bitrate(self):
        return self.data[1] & 0x3F

    @bitrate.setter
    def bitrate(self, value):
        self.data[1] = (self.data[1] & 0xC0) + value

    @property
    def prf(self):
        return (self.data[1] & 0xC0) >> 6

    @prf.setter
    def prf(self, value):
        self.data[1] = (self.data[1] & 0x3F) + (value << 6)

    @property
    def plen(self):
        return self.data[2]

    @plen.setter
    def plen(self, value):
        self.data[2] = value

    @property
    def gain_db(self):
        return float(self.data[3]) / 2

    @gain_db.setter
    def gain_db(self, value):
        self.data[3] = int(value * 2)


# TODO maybe change with properties one day?
class FilterData(ByteStructure):
    byte_size = 1
    data_format = 'B'

    def __init__(self, filter_type=0, filter_strength=0):
        self.data = [filter_type + (filter_strength << 4)]

    def load(self, data, convert=False):
        self.data = data

    def get_filter_name(self):
        filter_types = {
            PozyxConstants.FILTER_TYPE_NONE: "No filter",
            PozyxConstants.FILTER_TYPE_FIR: "FIR filter",
            PozyxConstants.FILTER_TYPE_MOVING_AVERAGE: "Moving average filter",
            PozyxConstants.FILTER_TYPE_MOVING_MEDIAN: "Moving median filter",
        }
        return filter_types.get(self.filter_type, "Unknown filter {}".format(self.filter_type))

    def __str__(self):
        return "{} with strength {}".format(self.get_filter_name(), self.filter_strength)
    
    @property
    def value(self):
        return self.data[0]
    
    @value.setter
    def value(self, value):
        self.data[0] = value

    @property
    def filter_type(self):
        return self.data[0] & 0xF

    @filter_type.setter
    def filter_type(self, value):
        self.data[0] = value + (self.filter_strength << 4)

    @property
    def filter_strength(self):
        return self.data[0] >> 4

    @filter_strength.setter
    def filter_strength(self, value):
        self.data[0] = self.filter_type + (value << 4)


class AlgorithmData(ByteStructure):
    byte_size = 1
    data_format = 'B'

    def __init__(self, algorithm=0, dimension=0):
        self.data = [algorithm + (dimension << 4)]

    def load(self, data, convert=False):
        self.data = data

    def get_algorithm_name(self):
        algorithms = {
            PozyxConstants.POSITIONING_ALGORITHM_UWB_ONLY: "UWB only",
            PozyxConstants.POSITIONING_ALGORITHM_TRACKING: "Tracking",
            PozyxConstants.POSITIONING_ALGORITHM_NONE: "None",
        }
        return algorithms.get(self.algorithm, "Unknown filter {}".format(self.algorithm))

    def get_dimension_name(self):
        dimensions = {
            PozyxConstants.DIMENSION_2D: "2D",
            PozyxConstants.DIMENSION_2_5D: "2.5D",
            PozyxConstants.DIMENSION_3D: "3D",
        }
        return dimensions.get(self.dimension, "Unknown filter {}".format(self.algorithm))

    def __str__(self):
        return "Algorithm {}, dimension {}".format(self.get_algorithm_name(), self.get_dimension_name())

    @property
    def value(self):
        return self.data[0]

    @value.setter
    def value(self, value):
        self.data[0] = value

    @property
    def algorithm(self):
        return self.data[0] & 0xF

    @algorithm.setter
    def algorithm(self, value):
        self.data[0] = value + (self.dimension << 4)

    @property
    def dimension(self):
        return self.data[0] >> 4

    @dimension.setter
    def dimension(self, value):
        self.data[0] = self.algorithm + (value << 4)
