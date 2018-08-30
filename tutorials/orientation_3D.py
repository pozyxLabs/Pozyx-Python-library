#!/usr/bin/env python
"""
The pozyx ranging demo (c) Pozyx Labs
please check out https://www.pozyx.io/Documentation/Tutorials/getting_started/Python

This demo requires one (or two) pozyx shields. It demonstrates the 3D orientation and the functionality
to remotely read register data from a pozyx device. Connect one of the Pozyx devices with USB and run this script.

This demo reads the following sensor data:
- pressure
- acceleration
- magnetic field strength
- angular velocity
- the heading, roll and pitch
- the quaternion rotation describing the 3D orientation of the device. This can be used to transform from the body coordinate system to the world coordinate system.
- the linear acceleration (the acceleration excluding gravity)
- the gravitational vector

The data can be viewed in the Processing sketch orientation_3D.pde
"""
from time import time

from pypozyx import SensorData, SingleRegister, POZYX_SUCCESS, get_first_pozyx_serial_port, PozyxSerial, get_serial_ports
from pypozyx.definitions.bitmasks import POZYX_INT_MASK_IMU
from pythonosc.osc_message_builder import OscMessageBuilder
from pythonosc.udp_client import SimpleUDPClient

from pypozyx.tools.version_check import perform_latest_version_check


class Orientation3D(object):
    """Reads out all sensor data from either a local or remote Pozyx"""

    def __init__(self, pozyx, osc_udp_client, remote_id=None):
        self.pozyx = pozyx
        self.remote_id = remote_id

        self.osc_udp_client = osc_udp_client

    def setup(self):
        """There is no specific setup functionality"""
        self.pozyx.printDeviceInfo(self.remote_id)

        self.current_time = time()

    def loop(self):
        """Gets new IMU sensor data"""
        sensor_data = SensorData()
        calibration_status = SingleRegister()
        if self.remote_id is not None or self.pozyx.checkForFlag(POZYX_INT_MASK_IMU, 0.01) == POZYX_SUCCESS:
            status = self.pozyx.getAllSensorData(sensor_data, self.remote_id)
            status &= self.pozyx.getCalibrationStatus(calibration_status, self.remote_id)
            if status == POZYX_SUCCESS:
                self.publishSensorData(sensor_data, calibration_status)

    def publishSensorData(self, sensor_data, calibration_status):
        """Makes the OSC sensor data package and publishes it"""
        self.msg_builder = OscMessageBuilder("/sensordata")
        self.msg_builder.add_arg(int(1000 * (time() - self.current_time)))
        self.current_time = time()
        self.addSensorData(sensor_data)
        self.addCalibrationStatus(calibration_status)
        self.osc_udp_client.send(self.msg_builder.build())

    def addSensorData(self, sensor_data):
        """Adds the sensor data to the OSC message"""
        self.addComponentsOSC(sensor_data.pressure)
        self.addComponentsOSC(sensor_data.acceleration)
        self.addComponentsOSC(sensor_data.magnetic)
        self.addComponentsOSC(sensor_data.angular_vel)
        self.addComponentsOSC(sensor_data.euler_angles)
        self.addComponentsOSC(sensor_data.quaternion)
        self.addComponentsOSC(sensor_data.linear_acceleration)
        self.addComponentsOSC(sensor_data.gravity_vector)

    def addComponentsOSC(self, component):
        """Adds a sensor data component to the OSC message"""
        for data in component.data:
            self.msg_builder.add_arg(float(data))

    def addCalibrationStatus(self, calibration_status):
        """Adds the calibration status data to the OSC message"""
        self.msg_builder.add_arg(calibration_status[0] & 0x03)
        self.msg_builder.add_arg((calibration_status[0] & 0x0C) >> 2)
        self.msg_builder.add_arg((calibration_status[0] & 0x30) >> 4)
        self.msg_builder.add_arg((calibration_status[0] & 0xC0) >> 6)


if __name__ == '__main__':
    # Check for the latest PyPozyx version. Skip if this takes too long or is not needed by setting to False.
    check_pypozyx_version = True
    if check_pypozyx_version:
        perform_latest_version_check()

    # shortcut to not have to find out the port yourself
    serial_port = get_first_pozyx_serial_port()
    if serial_port is None:
        print("No Pozyx connected. Check your USB cable or your driver!")
        quit()

    # remote device network ID
    remote_id = 0x6e66
    # whether to use a remote device. If on False, remote_id is set to None which means the local device is used
    remote = False
    if not remote:
        remote_id = None

    # configure if you want to route OSC to outside your localhost. Networking knowledge is required.
    ip = "127.0.0.1"
    network_port = 8888

    pozyx = PozyxSerial(serial_port)
    osc_udp_client = SimpleUDPClient(ip, network_port)

    orientation_3d = Orientation3D(pozyx, osc_udp_client, remote_id=remote_id)
    orientation_3d.setup()
    print("Set up Orientation 3D")
    while True:
        orientation_3d.loop()
