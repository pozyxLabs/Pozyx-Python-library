from pypozyx import PozyxConstants, UWBSettings, SingleRegister, DeviceList


def all_discovery_uwb_settings():
    for channel in PozyxConstants.ALL_UWB_CHANNELS:
        for bitrate in PozyxConstants.ALL_UWB_BITRATES:
            for prf in PozyxConstants.ALL_UWB_PRFS:
                for plen in PozyxConstants.ALL_UWB_PLENS:
                    yield UWBSettings(channel, bitrate, prf, plen, 33)


def get_device_list(pozyx):
    """"""
    device_list_size = SingleRegister()
    pozyx.getDeviceListSize(device_list_size)
    device_list = DeviceList(list_size=device_list_size[0])
    pozyx.getDeviceIds(device_list)
    return device_list


def discover_all_devices(pozyx):
    original_uwb_settings = UWBSettings()
    pozyx.getUWBSettings(original_uwb_settings)

    for uwb_settings in all_discovery_uwb_settings():
        pozyx.setUWBSettings(uwb_settings)

        pozyx.clearDevices()
        pozyx.doDiscoveryAll(slots=3, slot_duration=0.1)

        device_list = get_device_list(pozyx)
        if device_list:
            print("Found on {}".format(uwb_settings))
            for device_id in device_list:
                print("\t- {}".format(device_id))

    pozyx.setUWBSettings(original_uwb_settings)


if __name__ == '__main__':
    from pypozyx import PozyxSerial, get_first_pozyx_serial_port
    pozyx = PozyxSerial(get_first_pozyx_serial_port())

    discover_all_devices(pozyx)
