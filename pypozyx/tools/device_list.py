from pypozyx import SingleRegister, DeviceList, DeviceCoordinates, Coordinates


def all_device_coordinates_in_device_list(pozyx, remote_id=None):
    list_size = SingleRegister()
    status = pozyx.getDeviceListSize(list_size, remote_id=remote_id)

    if list_size.value == 0:
        # TODO investigate if valid?
        return

    device_list = DeviceList(list_size=list_size.value)
    status &= pozyx.getDeviceIds(device_list, remote_id=remote_id)

    for device_id in device_list:
        coordinates = Coordinates()
        pozyx.getDeviceCoordinates(device_id, coordinates, remote_id=remote_id)
        yield DeviceCoordinates(device_id, 0, pos=coordinates)


