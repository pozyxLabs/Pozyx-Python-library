#!/usr/bin/env python
"""
Pozyx basic troubleshooting (c) Pozyx Labs 2017

If you're experiencing trouble with Pozyx, this should be your first step to check for problems.
Please read the article on https://www.pozyx.io/Documentation/Tutorials/troubleshoot_basics/Python
"""

import sys

from pypozyx import PozyxSerial, get_first_pozyx_serial_port, PozyxConstants, POZYX_SUCCESS, UWBSettings, PozyxRegisters
from pypozyx.structures.device_information import DeviceDetails
from pypozyx.tools.discovery import all_discovery_uwb_settings
from pypozyx.tools.device_list import all_device_coordinates_in_device_list


class Device(object):
    def __init__(self, id_, uwb_settings):
        self._id = id_
        self.uwb_settings = uwb_settings

    def __hash__(self):
        s = "{}{}{}{}{}".format(self._id, self.uwb_settings.channel, self.uwb_settings.bitrate,
                                self.uwb_settings.prf, self.uwb_settings.plen)
        # print(s, hash(s))
        return hash(s)

    def __eq__(self, other):
        return self._id == other._id and self.uwb_settings == other.uwb_settings

    def __str__(self):
        return "ID {}, UWB {}".format("0x%0.4x" % self._id, self.uwb_settings)

    def __repr__(self):
        return str(self)


def device_check(pozyx, remote_id=None):
    # pozyx.resetSystem()

    system_details = DeviceDetails()
    status = pozyx.getDeviceDetails(system_details, remote_id=remote_id)
    if status != POZYX_SUCCESS:
        print("Unable to get device details for device with id 0x%0.4x" % remote_id)
        return

    if remote_id is None:
        print("Local %s with id 0x%0.4x" % (system_details.device_name, system_details.id))
    else:
        print("%s with id 0x%0.4x" % (system_details.device_name.capitalize(), system_details.id))

    print("\tWho am i: 0x%0.2x" % system_details.who_am_i)
    print("\tFirmware version: v%s" % system_details.firmware_version_string)
    print("\tHardware version: v%s" % system_details.hardware_version_string)
    print("\tSelftest result: %s" % system_details.selftest_string)
    print("\tError: 0x%0.2x" % system_details.error_code)
    print("\tError message: %s" % system_details.error_message)

    uwb_settings = UWBSettings()

    pozyx.getUWBSettings(uwb_settings, remote_id=remote_id)

    print("\tUWB settings: %s" % uwb_settings)

    print("\tTotal saved registers: %s" % pozyx.getNumRegistersSaved(remote_id=remote_id))
    saved_registers_to_check = PozyxRegisters.ALL_POSITIONING_REGISTERS + PozyxRegisters.ALL_UWB_REGISTERS

    saved_registers = pozyx.getSavedRegisters(remote_id=remote_id)
    if saved_registers:
        # if system_details.device_name == "tag":
        print("\t\t- Positioning registers: ")
        for register, register_name in zip(PozyxRegisters.ALL_POSITIONING_REGISTERS, ["Filter", "Algorithm & Dimension", "Ranging protocol", "Height"]):
            byte_num = int(register / 8)
            bit_num = register % 8
            register_saved = bool(saved_registers[byte_num] >> bit_num & 0x1)
            print("\t\t\t- {}: {}".format(register_name, register_saved))
        print("\t\t- UWB registers: ")
        for register, register_name in zip(PozyxRegisters.ALL_UWB_REGISTERS, ["Channel", "Bitrate & PRF", "Preamble length", "Gain"]):
            byte_num = int(register / 8)
            bit_num = register % 8
            register_saved = bool(saved_registers[byte_num] >> bit_num & 0x1)
            print("\t\t\t- {}: {}".format(register_name, register_saved))
    else:
        print("\tFailed to retrieve saved registers.")


    if system_details.device_name == "tag" and remote_id is not None:
        print("\tDevices for positioning:")
        pozyx.printDeviceList(remote_id=remote_id, include_coordinates=True, prefix="\t\t-")


def network_check_discovery(pozyx, remote_id=None):
    pozyx.clearDevices(remote_id)
    if pozyx.doDiscovery(discovery_type=PozyxConstants.DISCOVERY_ALL_DEVICES, remote_id=remote_id) == POZYX_SUCCESS:
        print("Found devices:")
        pozyx.printDeviceList(remote_id, include_coordinates=False)


def discover_all_devices(pozyx):
    amount_of_settings = len([uwb for uwb in all_discovery_uwb_settings()])
    original_uwb_settings = UWBSettings()
    pozyx.getUWBSettings(original_uwb_settings)
    devices = set()

    print('\nStarting discovery')
    print('------------------')
    sys.stdout.write("Discovery progress [%s]" % (" " * amount_of_settings))
    sys.stdout.flush()
    sys.stdout.write("\b" * (amount_of_settings + 1))  # return to start of line, after '['

    counter = 0
    s = "["
    for uwb_settings in all_discovery_uwb_settings():
        pozyx.setUWBSettings(uwb_settings)
        for i in range(2):
            pozyx.clearDevices()
            pozyx.doDiscovery(PozyxConstants.DISCOVERY_ALL_DEVICES)

            for device_coordinates in all_device_coordinates_in_device_list(pozyx):
                uwb_device = UWBSettings()
                pozyx.getUWBSettings(uwb_device, remote_id=device_coordinates.network_id)
                device = Device(device_coordinates.network_id, uwb_device)
                # print(device, hash(device))
                # print(device in devices)
                devices.add(device)

        sys.stdout.write("=")
        sys.stdout.flush()

    sys.stdout.write("]\n\nDiscovery finished.\n")
    for device in devices:
        pozyx.setUWBSettings(device.uwb_settings)
        device_check(pozyx, device._id)
        print()

    pozyx.setUWBSettings(original_uwb_settings)

    # print(devices)


if __name__ == '__main__':
    serial_port = get_first_pozyx_serial_port()
    if serial_port is None:
        print("No Pozyx connected. Check your USB cable or your driver!")
        quit()

    pozyx = PozyxSerial(serial_port)

    # change to remote ID for troubleshooting that device
    remote_id = None

    device_check(pozyx, remote_id)
    # network_check_discovery(pozyx, remote_id)
    discover_all_devices(pozyx)
