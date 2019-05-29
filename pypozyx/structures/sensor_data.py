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


class PositionError(ByteStructure):
    """Container for position error in x, y, z, xy, xz, and yz (in mm)."""
    physical_convert = 1
    byte_size = 12
    data_format = 'hhhhhh'

    def __init__(self, x=0, y=0, z=0, xy=0, xz=0, yz=0):
        """Initializes the PositionError object."""
        self.data = [x, y, z, xy, xz, yz]

    def load(self, data, convert=False):
        self.data = data

    def __str__(self):
        return 'X: {}, Y: {}, Z: {}, XY: {}, XZ: {}, YZ: {}'.format(self.x, self.y, self.z, self.xy, self.xz, self.yz)

    @property
    def x(self):
        return self.data[0] / self.physical_convert

    @x.setter
    def x(self, value):
        self.data[0] = value * self.physical_convert

    @property
    def y(self):
        return self.data[1] / self.physical_convert

    @y.setter
    def y(self, value):
        self.data[1] = value * self.physical_convert

    @property
    def z(self):
        return self.data[2] / self.physical_convert

    @z.setter
    def z(self, value):
        self.data[2] = value * self.physical_convert

    @property
    def xy(self):
        return self.data[3] / self.physical_convert

    @xy.setter
    def xy(self, value):
        self.data[3] = value * self.physical_convert

    @property
    def xz(self):
        return self.data[4] / self.physical_convert

    @xz.setter
    def xz(self, value):
        self.data[4] = value * self.physical_convert

    @property
    def yz(self):
        return self.data[5] / self.physical_convert

    @yz.setter
    def yz(self, value):
        self.data[5] = value * self.physical_convert


class Quaternion(ByteStructure):
    """Container for quaternion data in x, y, z and w."""
    physical_convert = PozyxConstants.QUATERNION_DIV

    byte_size = 8
    data_format = 'hhhh'

    def __init__(self, w=0, x=0, y=0, z=0):
        """Initializes the Quaternion object."""
        self.data = [w, x, y, z]

    def load(self, data, convert=True):
        for i in range(len(data)):
            data[i] = float(data[i])
        self.data = data

    def get_sum(self):
        """Returns the normalization value of the quaternion"""
        return sqrt((self.x * self.x) + (self.y * self.y) + (self.z * self.z) + (self.w * self.w))

    def normalize(self):
        """Normalizes the quaternion's data"""
        sum = self.get_sum()
        for i in range(len(self.data)):
            self.data[i] /= sum

    def __str__(self):
        return 'X: {self.x}, Y: {self.y}, Z: {self.z}, W: {self.w}'.format(self=self)

    @property
    def w(self):
        return self.data[0] / self.physical_convert

    @w.setter
    def w(self, value):
        self.data[0] = value * self.physical_convert

    @property
    def x(self):
        return self.data[1] / self.physical_convert

    @x.setter
    def x(self, value):
        self.data[1] = value * self.physical_convert

    @property
    def y(self):
        return self.data[2] / self.physical_convert

    @y.setter
    def y(self, value):
        self.data[2] = value * self.physical_convert

    @property
    def z(self):
        return self.data[3] / self.physical_convert

    @z.setter
    def z(self, value):
        self.data[3] = value * self.physical_convert

    def to_dict(self):
        return {
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "w": self.w,
        }


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

    def load(self, data, convert=True):
        self.data = data

    def __str__(self):
        return 'Heading: {}, Roll: {}, Pitch: {}'.format(self.heading, self.roll, self.pitch)

    @property
    def heading(self):
        return self.data[0] / self.physical_convert

    @heading.setter
    def heading(self, value):
        self.data[0] = value * self.physical_convert

    @property
    def roll(self):
        return self.data[1] / self.physical_convert

    @roll.setter
    def roll(self, value):
        self.data[1] = value * self.physical_convert

    @property
    def pitch(self):
        return self.data[2] / self.physical_convert

    @pitch.setter
    def pitch(self, value):
        self.data[2] = value * self.physical_convert


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

    def __init__(self, data=None):
        """Initializes the RawSensorData object"""
        data = [0] * 24 if data is None else data
        SensorData.__init__(self, data)

    def load(self, data, convert=False):
        SensorData.load(self, data, convert=False)


class CoordinatesWithStatus(ByteStructure):
    """Container for coordinates in x, y, and z (in mm)."""
    byte_size = 13
    data_format = 'iiiB'

    def __init__(self, x=0, y=0, z=0, status=0):
        """Initializes the XYZ or XYZ-derived object."""
        self.data = [x, y, z, status]

    def load(self, data, convert=False):
        self.data = data

    def __str__(self):
        return 'STATUS: {}, X: {}, Y: {}, Z: {}'.format(self.status, self.x, self.y, self.z)

    @property
    def x(self):
        return self.data[0]

    @x.setter
    def x(self, value):
        self.data[0] = value

    @property
    def y(self):
        return self.data[1]

    @y.setter
    def y(self, value):
        self.data[1] = value

    @property
    def z(self):
        return self.data[2]

    @z.setter
    def z(self, value):
        self.data[2] = value

    @property
    def status(self):
        return self.data[3]

    @status.setter
    def status(self, value):
        self.data[3] = value

    def to_dict(self):
        return {
            "x": self.x,
            "y": self.y,
            "z": self.z,
        }


class RangeInformation(ByteStructure):
    byte_size = 6
    data_format = 'HI'

    def __init__(self, device_id=0, distance=0):
        self.data = [device_id, distance]

    def load(self, data, convert=0):
        self.data = data

    def __str__(self):
        return "0x{:04x}: {} mm".format(self.device_id, self.distance)

    @property
    def device_id(self):
        return self.data[0]

    @device_id.setter
    def device_id(self, value):
        self.data[0] = value

    @property
    def distance(self):
        return self.data[1]

    @distance.setter
    def distance(self, value):
        self.data[1] = value


class PositioningData(ByteStructure):
    SENSOR_ORDER = [CoordinatesWithStatus, Acceleration, AngularVelocity, Magnetic, EulerAngles, Quaternion,
                    LinearAcceleration, Acceleration, Pressure, MaxLinearAcceleration]

    def __init__(self, flags):
        self.data = [0]
        self.flags = flags
        self.containers = []
        self.byte_size = 0
        self.data_format = ''
        self.amount_of_ranges = 0

        self.set_data_structures()

    def load(self, data, convert=0):
        self.data = data
        data_index = 0
        for container in self.containers:
            data_length = len(container.data_format)
            container.load(data[data_index:data_index + data_length])
            data_index += data_length

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

    # TODO
    def __str__(self):
        return 'TODO'
