#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""pypozyx.structures.sensor_data - Contains container classes for data from the Pozyx's many sensors."""
from math import sqrt

from pypozyx.definitions.constants import PozyxConstants
from pypozyx.structures.byte_structure import ByteStructure
from pypozyx.structures.generic import XYZ, SingleSensorValue, SingleRegister


class Coordinates(XYZ):
    """Container for coordinates in x, y, and z (in mm)."""
    byte_size = 12
    data_format = 'iii'

    def load(self, data, convert=False):
        self.data = data
        self.x = data[0]
        self.y = data[1]
        self.z = data[2]


class Acceleration(XYZ):
    """Container for acceleration in x, y, and z (in mg)."""
    physical_convert = PozyxConstants.ACCELERATION_DIV_MG

    byte_size = 6
    data_format = 'hhh'


class Magnetic(XYZ):
    """Container for coordinates in x, y, and z (in uT)."""
    physical_convert = PozyxConstants.MAGNETOMETER_DIV_UT

    byte_size = 6
    data_format = 'hhh'


class AngularVelocity(XYZ):
    """Container for angular velocity in x, y, and z (in dps)."""
    physical_convert = PozyxConstants.GYRO_DIV_DPS

    byte_size = 6
    data_format = 'hhh'


class LinearAcceleration(XYZ):
    """Container for linear acceleration in x, y, and z (in mg), as floats."""
    physical_convert = PozyxConstants.ACCELERATION_DIV_MG

    byte_size = 6
    data_format = 'hhh'

    def load(self, data, convert=True):
        if convert:
            self.x = float(data[0]) / self.physical_convert
            self.y = float(data[1]) / self.physical_convert
            self.z = float(data[2]) / self.physical_convert
        else:
            self.x = float(data[0])
            self.y = float(data[1])
            self.z = float(data[2])


class PositionError(XYZ):
    """Container for position error in x, y, z, xy, xz, and yz (in mm)."""
    physical_convert = 1
    byte_size = 12
    data_format = 'hhhhhh'

    def __init__(self, x=0, y=0, z=0, xy=0, xz=0, yz=0):
        """Initializes the PositionError object."""
        XYZ.__init__(self, x, y, z)
        self.xy = xy
        self.xz = xz
        self.yz = yz
        self.data = [x, y, z, xy, xz, yz]

    def load(self, data):
        XYZ.load(self, data[0:3])
        self.xy = data[3] / self.physical_convert
        self.xz = data[4] / self.physical_convert
        self.yz = data[5] / self.physical_convert

    def update_data(self):
        try:
            if self.data != [self.x, self.y, self.z, self.xy, self.xz, self.yz]:
                self.data = [self.x, self.y, self.z, self.xy, self.xz, self.yz]
        except:
            return

    def __str__(self):
        return 'X: {self.x}, Y: {self.y}, Z: {self.z}, XY: {self.xy}, XZ: {self.xz}, YZ: {self.yz}'.format(self=self)


class Quaternion(XYZ):
    """Container for quaternion data in x, y, z and w."""
    physical_convert = PozyxConstants.QUATERNION_DIV

    byte_size = 8
    data_format = 'hhhh'

    def __init__(self, w=0, x=0, y=0, z=0):
        """Initializes the Quaternion object."""
        XYZ.__init__(self, x, y, z)
        self.data = [w, x, y, z]
        self.w = w

    def load(self, data, convert=True):
        for i in range(len(data)):
            data[i] = float(data[i])
        XYZ.load(self, data[1:4], convert)
        self.data = data
        if convert:
            self.w = data[0] / self.physical_convert
        else:
            self.w = data[0]

    def update_data(self):
        try:
            if self.data != [self.w, self.x, self.y, self.z]:
                self.data = [self.w, self.x, self.y, self.z]
        except:
            return

    def get_sum(self):
        """Returns the normalization value of the quaternion"""
        return sqrt((self.x * self.x) + (self.y * self.y) + (self.z * self.z) + (self.w * self.w))

    def normalize(self):
        """Normalizes the quaternion's data"""
        sum = self.get_sum()
        self.w /= sum
        self.x /= sum
        self.y /= sum
        self.z /= sum

    def __str__(self):
        return 'X: {self.x}, Y: {self.y}, Z: {self.z}, W: {self.w}'.format(self=self)


class MaxLinearAcceleration(SingleSensorValue):
    physical_convert = PozyxConstants.MAX_LINEAR_ACCELERATION_DIV_MG

    byte_size = 2
    data_format = 'h'


class Temperature(SingleSensorValue):
    physical_convert = PozyxConstants.TEMPERATURE_DIV_CELSIUS

    byte_size = 1
    data_format = 'b'

    def __str__(self):
        return "{self.value} °C"


class Pressure(SingleSensorValue):
    physical_convert = PozyxConstants.PRESSURE_DIV_PA

    byte_size = 4
    data_format = 'I'


class EulerAngles(ByteStructure):
    """Container for euler angles as heading, roll, and pitch (in degrees)."""
    physical_convert = PozyxConstants.EULER_ANGLES_DIV_DEG

    byte_size = 6
    data_format = 'hhh'

    def __init__(self, heading=0, roll=0, pitch=0):
        """Initializes the EulerAngles object."""
        self.data = [heading, roll, pitch]
        self.heading = heading
        self.roll = roll
        self.pitch = pitch

    def load(self, data, convert=True):
        self.data = data
        if convert:
            self.heading = float(data[0]) / self.physical_convert
            self.roll = float(data[1]) / self.physical_convert
            self.pitch = float(data[2]) / self.physical_convert
        else:
            self.heading = float(data[0])
            self.roll = float(data[1])
            self.pitch = float(data[2])

    def update_data(self):
        try:
            if self.data != [self.heading, self.roll, self.pitch]:
                self.data = [self.heading, self.roll, self.pitch]
        except:
            return

    def __str__(self):

        return 'Heading: {self.heading}, Roll: {self.roll}, Pitch: {self.pitch}'.format(
            self=self)


class SensorData(ByteStructure):
    """
    Container for all sensor data.

    This includes, in order, with respective structure:
        - pressure : UInt32
        - acceleration : Acceleration
        - magnetic : Magnetic
        - angular_vel : AngularVelocity
        - euler_angles : EulerAngles
        - quaternion : Quaternion
        - linear_acceleration: LinearAcceleration
        - gravity_vector: LinearAcceleration
        - temperature: Int8
    """
    byte_size = 49  # 4 + 6 + 6 + 6 + 6 + 8 + 6 + 6 + 1

    # 'I' + 'h'*3 + 'h'*3 + 'h'*3 + 'h'*3 + 'f'*4 + 'h'*3 + 'h'*3 + 'b'
    data_format = 'Ihhhhhhhhhhhhhhhhhhhhhhb'

    def __init__(self, data=[0] * 24):
        """Initializes the SensorData object."""
        self.data = data
        self.pressure = Pressure(data[0])
        self.acceleration = Acceleration(data[1], data[2], data[3])
        self.magnetic = Magnetic(data[4], data[5], data[6])
        self.angular_vel = AngularVelocity(data[7], data[8], data[9])
        self.euler_angles = EulerAngles(data[10], data[11], data[12])
        self.quaternion = Quaternion(data[13], data[14], data[15], data[16])
        self.linear_acceleration = LinearAcceleration(
            data[17], data[18], data[19])
        self.gravity_vector = LinearAcceleration(data[20], data[21], data[22])
        self.temperature = Temperature(data[23])

    def load(self, data, convert=True):
        self.data = data
        self.pressure.load([data[0]], convert)
        self.acceleration.load(data[1:4], convert)
        self.magnetic.load(data[4:7], convert)
        self.angular_vel.load(data[7:10], convert)
        self.euler_angles.load(data[10:13], convert)
        self.quaternion.load(data[13:17], convert)
        self.linear_acceleration.load(data[17:20], convert)
        self.gravity_vector.load(data[20:23], convert)
        self.temperature.load([data[23]], convert)

    def update_data(self):
        """Not used so data remains the raw unformatted data"""
        pass


class RawSensorData(SensorData):
    """Container for raw sensor data

    This includes, in order, with respective structure:
        - pressure : UInt32
        - acceleration : Acceleration
        - magnetic : Magnetic
        - angular_vel : AngularVelocity
        - euler_angles : EulerAngles
        - quaternion : Quaternion
        - linear_acceleration: LinearAcceleration
        - gravity_vector: LinearAcceleration
        - temperature: Int8
    """

    def __init__(self, data=[0] * 24):
        """Initializes the RawSensorData object"""
        SensorData.__init__(self, data)

    def load(self, data):
        SensorData.load(self, data, convert=False)


class CoordinatesWithStatus(ByteStructure):
    """Container for coordinates in x, y, and z (in mm)."""
    byte_size = 13
    data_format = 'iiiB'

    def __init__(self, x=0, y=0, z=0, status=False):
        """Initializes the XYZ or XYZ-derived object."""
        self.x = x
        self.y = y
        self.z = z
        self.status = status
        self.data = [x, y, z, status]

    def load(self, data, convert=False):
        self.data = data
        self.x = data[0]
        self.y = data[1]
        self.z = data[2]
        self.status = data[3]

    def update_data(self):
        try:
            if self.data != [self.x, self.y, self.z, self.status]:
                self.data = [self.x, self.y, self.z, self.status]
        except:
            return

    def __str__(self):
        return 'STATUS: {self.status}, X: {self.x}, Y: {self.y}, Z: {self.z}'.format(self=self)


class RangeInformation(ByteStructure):
    byte_size = 6
    data_format = 'HI'

    def __init__(self, device_id=0, distance=0):
        self.device_id = device_id
        self.distance = distance
        self.data = [device_id, distance]

    def load(self, data, convert=0):
        self.data = data
        self.device_id = data[0]
        self.distance = data[1]

    def update_data(self):
        try:
            if self.data != [self.device_id, self.distance]:
                self.data = [self.device_id, self.distance]
        except:
            return

    def __str__(self):
        return "0x%0.4x: %d mm" % (self.device_id, self.distance)


class PositioningData(ByteStructure):
    SENSOR_ORDER = [CoordinatesWithStatus, Acceleration, AngularVelocity, Magnetic, EulerAngles, Quaternion,
                    LinearAcceleration, Acceleration, Pressure, MaxLinearAcceleration]

    def __init__(self, flags):
        self.data = [0]
        self.flags = flags
        self.containers = []
        self.byte_size = 0
        self.data_format = ''

        self.set_data_structures()

    def add_sensor(self, sensor_class):
        self.byte_size += sensor_class.byte_size
        self.data_format += sensor_class.data_format
        self.containers.append(sensor_class())

    def set_data_structures(self):
        self.containers = []
        self.byte_size = 0
        self.data_format = ''

        for index, sensor in enumerate(self.SENSOR_ORDER):
            if self.flags & (1 << index):
                self.add_sensor(sensor)

        if self.has_ranges():
            self.byte_size += 1
            self.data_format += 'B'
            self.containers.append(SingleRegister())

    def set_amount_of_ranges(self, amount_of_ranges):
        self.amount_of_ranges = amount_of_ranges
        if self.has_ranges():
            for i in range(amount_of_ranges):
                self.add_sensor(RangeInformation)

    def has_ranges(self):
        return self.flags & (1 << 15)

    def load_bytes(self, byte_data):
        """Loads a hex byte array in the structure's data"""
        self.byte_data = byte_data
        self.bytes_to_data()

    @property
    def number_of_ranges(self):
        if self.has_ranges():
            return self.amount_of_ranges
        else:
            # TODO right error?
            raise ValueError("Ranges not given as a flag")


    def load(self, data, convert=0):
        self.data = data
        data_index = 0
        for container in self.containers:
            data_length = len(container.data_format)
            container.load(data[data_index:data_index + data_length])
            data_index += data_length

    # TODO
    def __str__(self):
        return 'TODO'
