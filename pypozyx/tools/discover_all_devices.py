from pypozyx import PozyxConstants, UWBSettings, SingleRegister, DeviceList, DeviceCoordinates, Coordinates


def all_discovery_uwb_settings():
    for channel in PozyxConstants.ALL_UWB_CHANNELS:
        for bitrate in PozyxConstants.ALL_UWB_BITRATES:
            for prf in PozyxConstants.ALL_UWB_PRFS:
                for plen in [PozyxConstants.UWB_PLEN_64, PozyxConstants.UWB_PLEN_1536]:
                    yield UWBSettings(channel, bitrate, prf, plen, 33)


def all_device_coordinates_in_device_list(pozyx, remote_id=None):
    list_size = SingleRegister()
    status = pozyx.getDeviceListSize(list_size, remote_id=remote_id)

    if list_size.value == 0:
        return

    device_list = DeviceList(list_size=list_size.value)
    status &= pozyx.getDeviceIds(device_list, remote_id=remote_id)

    for device_id in device_list:
        coordinates = Coordinates()
        pozyx.getDeviceCoordinates(device_id, coordinates, remote_id=remote_id)
        yield DeviceCoordinates(device_id, 0, pos=coordinates)

