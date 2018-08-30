from pypozyx.definitions.constants import PozyxConstants, ERROR_MESSAGES
from pypozyx.structures.byte_structure import ByteStructure


class DeviceDetails(ByteStructure):
    """Container for system information, such as firmware, hardware...

    Read-only, so all values are implemented as properties.
    """

    byte_size = 5
    data_format = 'BBBBB'

    def __init__(self, network_id=None):
        super(DeviceDetails, self).__init__()

        self.id = network_id

    def load(self, data, convert=True):
        self.data = data

    @property
    def firmware_version(self):
        return self.data[1]

    @property
    def hardware_version(self):
        return self.data[2]

    @property
    def who_am_i(self):
        return self.data[0]

    @property
    def selftest(self):
        return self.data[3]

    @property
    def error_code(self):
        return self.data[4]

    @property
    def error_message(self):
        return ERROR_MESSAGES[self.error_code]

    @property
    def firmware_version_major(self):
        return self.firmware_version >> 4

    @property
    def firmware_version_minor(self):
        return self.firmware_version & 0xF

    @property
    def firmware_version_string(self):
        return "{}.{}".format(self.firmware_version_major, self.firmware_version_minor)

    @property
    def device_type(self):
        return self.data[2] >> 5

    @property
    def device_name(self):
        if self.device_type == PozyxConstants.TAG_MODE:
            return "tag"
        return "anchor"

    @property
    def hardware_version_string(self):
        return "1.{}".format(self.hardware_version & 0x1F)

    @property
    def device_string(self):
        if self.id is None:
            return "None"
        return "0x%0.4x" % self.id

    @property
    def selftest_string(self):
        return bin(self.selftest)


class PositioningSettings(object):
    pass


class DeviceInformation(object):
    pass