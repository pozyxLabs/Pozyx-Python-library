from pypozyx.definitions.registers import *
from pypozyx.definitions.constants import *
from pypozyx.definitions.bitmasks import *

from pypozyx.core import PozyxCore

from pypozyx.structures.generic import Data, SingleRegister, dataCheck
from pypozyx.structures.device import *
from pypozyx.structures.sensor_data import *

import time


class PozyxLib(PozyxCore):

    def __init__(self):
        pass

    def getRead(self, address, data, remote_id=None):
        if remote_id is None:
            return self.regRead(address, data)
        else:
            return self.remoteRegRead(remote_id, address, data)

    def useFunction(self, function, params=Data([]), data=Data([]), remote_id=None):
        if remote_id is None:
            status = self.regFunction(function, params, data)
        else:
            status = self.remoteRegFunction(remote_id, function, params, data)
        return status

    def setWrite(self, address, data, remote_id=None, local_delay=POZYX_DELAY_LOCAL_WRITE, remote_delay=POZYX_DELAY_REMOTE_WRITE):
        if remote_id is None:
            status = self.regWrite(address, data)
            time.sleep(local_delay)
        else:
            status = self.remoteRegWrite(remote_id, address, data)
            time.sleep(remote_delay)
        return status

    def getWhoAmI(self, whoami, remote_id=None):
        return self.getRead(POZYX_WHO_AM_I, whoami, remote_id)

    def getFirmwareVersion(self, firmware, remote_id=None):
        return self.getRead(POZYX_FIRMWARE_VER, firmware, remote_id)

    def getHardwareVersion(self, hardware, remote_id=None):
        return self.getRead(POZYX_HARDWARE_VER, hardware, remote_id)

    def getSelftest(self, selftest, remote_id=None):
        return self.getRead(POZYX_ST_RESULT, selftest, remote_id)

    def getErrorCode(self, error_code, remote_id=None):
        return self.getRead(POZYX_ERRORCODE, error_code, remote_id)

    def getInterruptStatus(self, interrupts, remote_id=None):
        return self.getRead(POZYX_INT_STATUS, interrupts, remote_id)

    def getCalibrationStatus(self, calibration_status, remote_id=None):
        return self.getRead(POZYX_CALIB_STATUS, calibration_status, remote_id)

    def getInterruptMask(self, mask, remote_id=None):
        return self.getRead(POZYX_INT_MASK, mask, remote_id)

    def getUpdateInterval(self, ms, remote_id=None):
        return self.getRead(POZYX_POS_INTERVAL, ms, remote_id)

    def getPositionAlgorithm(self, algorithm, remote_id=None):
        status = self.getRead(POZYX_POS_ALG, algorithm, remote_id)
        algorithm[0] &= 0xF
        return status

    def getPositionDimension(self, dimension, remote_id=None):
        status = self.getRead(POZYX_POS_ALG, dimension, remote_id)
        dimension[0] = (dimension[0] & 0x30) >> 4
        return status

    def getAnchorSelectionMode(self, mode, remote_id=None):
        status = self.getRead(POZYX_POS_NUM_ANCHORS, mode, remote_id)
        mode[0] = (mode[0] & 0x80) >> 7
        return status

    def getNumberOfAnchors(self, nr_anchors, remote_id=None):
        status = self.getRead(POZYX_POS_NUM_ANCHORS, nr_anchors, remote_id)
        nr_anchors[0] &= 0xF
        return status

    def getNetworkId(self, network_id):
        return self.regRead(POZYX_NETWORK_ID, network_id)

    def getUWBSettings(self, UWB_settings, remote_id=None):
        # The UWB data register size is 4.
        tmp_data = Data([0] * 4)
        status = self.getRead(POZYX_UWB_CHANNEL, tmp_data, remote_id)
        UWB_settings.load(tmp_data.data)
        return status

    def getUWBChannel(self, channel_num, remote_id=None):
        channel_num[0] = 0
        status = self.getRead(POZYX_UWB_CHANNEL, channel_num, remote_id)
        if channel_num[0] == 0 or status == POZYX_FAILURE:
            return POZYX_FAILURE
        return status

    def getTxPower(self, txgain_dB, remote_id=None):
        doublegain_dB = SingleRegister()
        status = self.getRead(
            POZYX_UWB_GAIN, doublegain_dB, remote_id)
        txgain_dB[0] = 0.5 * doublegain_dB[0]
        return status

    def getOperationMode(self, mode, remote_id=None):
        return self.getRead(POZYX_OPERATION_MODE, mode, remote_id)

    def getSensorMode(self, sensor_mode, remote_id=None):
        return self.getRead(POZYX_SENSORS_MODE, sensor_mode, remote_id)

    def getCoordinates(self, coordinates, remote_id=None):
        status = self.getRead(POZYX_POS_X, coordinates, remote_id)
        return status

    def getPositionError(self, pos_error, remote_id=None):
        status = self.getRead(POZYX_POS_ERR_X, pos_error, remote_id)
        return status

    def getAllSensorData(self, sensor_data, remote_id=None):
        status = self.getRead(POZYX_PRESSURE, sensor_data, remote_id)
        return status

    def getPressure_Pa(self, pressure, remote_id=None):
        status = self.getRead(POZYX_PRESSURE, pressure, remote_id)
        pressure[0] = pressure[0] / POZYX_PRESS_DIV_PA
        return status

    def getAcceleration_mg(self, acceleration, remote_id=None):
        status = self.getRead(POZYX_ACCEL_X, acceleration, remote_id)
        return status

    def getMagnetic_uT(self, magnetic, remote_id=None):
        status = self.getRead(POZYX_MAGN_X, magnetic, remote_id)
        return status

    def getAngularVelocity_dps(self, angular_vel, remote_id=None):
        status = self.getRead(POZYX_GYRO_X, angular_vel, remote_id)
        return status

    def getEulerAngles_deg(self, euler_angles, remote_id=None):
        status = self.getRead(
            POZYX_EUL_HEADING, euler_angles, remote_id)
        return status

    def getQuaternion(self, quaternion, remote_id=None):
        status = self.getRead(POZYX_QUAT_W, quaternion, remote_id)
        return status

    def getLinearAcceleration_mg(self, linear_acceleration, remote_id=None):
        status = self.getRead(
            POZYX_LIA_X, linear_acceleration, remote_id)
        return status

    def getGravityVector_mg(self, gravity_vector, remote_id=None):
        status = self.getRead(
            POZYX_GRAV_X, gravity_vector, remote_id)
        return status

    def getTemperature_c(self, temperature, remote_id=None):
        status = self.getRead(
            POZYX_TEMPERATURE, temperature, remote_id)
        temperature[0] = data[0] / POZYX_TEMP_DIV_CELSIUS
        return status

    def getDeviceListSize(self, device_list_size, remote_id=None):
        return self.getRead(POZYX_DEVICE_LIST_SIZE, device_list_size, remote_id)

    def getLastNetworkId(self, network_id, remote_id=None):
        return self.getRead(POZYX_RX_NETWORK_ID, network_id, remote_id)

    def getLastDataLength(self, data_length, remote_id=None):
        return self.getRead(POZYX_RX_DATA_LEN, data_length, remote_id)

    def getConfigModeGPIO(self, gpio_num, mode, remote_id=None):
        assert gpio_num > 0 or gpio_num <= 4, 'getConfigModeGPIO: GPIO number incorrect'
        gpio_register = POZYX_CONFIG_GPIO1 + gpio_num - 1
        status = self.getRead(gpio_register, mode, remote_id)
        mode[0] &= 0x7
        return status

    def getConfigPullGPIO(self, gpio_num, pull, remote_id=None):
        assert gpio_num > 0 or gpio_num <= 4, 'getConfigPullGPIO: GPIO number incorrect'
        gpio_register = POZYX_CONFIG_GPIO1 + gpio_num - 1
        status = self.getRead(gpio_register, pull, remote_id)
        pull[0] = (pull[0] & 0x18) >> 3
        return status

    def getGPIO(self, gpio_num, value, remote_id=None):
        assert gpio_num > 0 or gpio_num <= 4, 'getGPIO: GPIO number incorrect'
        gpio_register = POZYX_CONFIG_GPIO1 + gpio_num - 1
        return self.getRead(gpio_register, value, remote_id)

    def getSystemError(self, remote_id=None):
        error_code = SingleRegister()
        self.getRead(POZYX_ERRORCODE, error_code, remote_id)

        error_codes = {POZYX_ERROR_NONE: "",
                       POZYX_ERROR_I2C_WRITE: "Error 0x01: Error writing to a register through the I2C bus",
                       POZYX_ERROR_I2C_CMDFULL: "Error 0x02: Pozyx cannot handle all the I2C commands at once",
                       POZYX_ERROR_ANCHOR_ADD: "Error 0x03: Cannot add anchor to the internal device list",
                       POZYX_ERROR_COMM_QUEUE_FULL: "Error 0x04: Communication queue is full, too many UWB messages",
                       POZYX_ERROR_I2C_READ: "Error 0x05: Error reading from a register from the I2C bus",
                       POZYX_ERROR_UWB_CONFIG: "Error 0x06: Cannot change the UWB configuration",
                       POZYX_ERROR_OPERATION_QUEUE_FULL: "Error 0x07: Pozyx cannot handle all the operations at once",
                       POZYX_ERROR_STARTUP_BUSFAULT: "Error 0x08: Internal bus error",
                       POZYX_ERROR_FLASH_INVALID: "Error 0x09: Flash memory is corrupted or invalid",
                       POZYX_ERROR_NOT_ENOUGH_ANCHORS: "Error 0x0A: Not enough anchors available for positioning",
                       POZYX_ERROR_DISCOVERY: "Error 0x0B: Error during the Discovery process",
                       POZYX_ERROR_CALIBRATION: "Error 0x0C: Error during the auto calibration process",
                       POZYX_ERROR_FUNC_PARAM: "Error 0x0D: Invalid function parameters for the register function",
                       POZYX_ERROR_ANCHOR_NOT_FOUND: "Error 0x0E: The coordinates of an anchor are not found",
                       POZYX_ERROR_GENERAL: "Error 0xFF: General error"}

        return error_codes.get(error_code[0], "Unknown error")

    def getPositioningAnchorIds(self, anchors, remote_id=None):
        assert len(anchors) > 0 and len(
            anchors) <= 10, 'getPositioningAnchorIds: Anchor number out of range'
        device_list_size = SingleRegister()
        status = self.getDeviceListSize(device_list_size, remote_id)
        if len(anchors) < device_list_size[0] or status == POZYX_FAILURE:
            return POZYX_FAILURE
        return self.useFunction(POZYX_POS_GET_ANCHOR_IDS, Data([]), anchors, remote_id)

    def getDeviceIds(self, devices, remote_id=None):
        assert len(devices) > 0 and len(
            devices) <= 20, 'getDeviceIds: size not in range'

        list_size = SingleRegister()

        status = self.getDeviceListSize(list_size, remote_id)
        if list_size[0] < len(devices) or status == POZYX_FAILURE:
            return POZYX_FAILURE

        params = Data([0, list_size[0]])

        self.useFunction(
            POZYX_DEVICES_GETIDS, params, devices, remote_id)

    def getAnchorIds(self, anchors, size, remote_id=None):
        assert len(anchors) > 0 and len(
            anchors) <= 20, 'getAnchorIds: size not in range'
        list_size = SingleRegister()
        status = self.getDeviceListSize(list_size, remote_id)
        if list_size[0] < len(anchors) or status == POZYX_FAILURE:
            return POZYX_FAILURE
        devices = DeviceList(list_size=list_size)
        status = self.useFunction(
            POZYX_DEVICES_GETIDS, Data([]), devices, remote_id)

        if status == POZYX_SUCCESS:
            for i in range(len(anchors)):
                anchors[i] = 0x0
            j = 0
            for i in range(list_size[0]):
                data = Data([0] * 3)
                status &= self.useFunction(
                    POZYX_DEVICE_GETINFO, NetworkID(devices[i]), data)
                # why + 1?
                if data[2] == POZYX_ANCHOR_MODE + 1:
                    anchors[j] = devices[i]
                    j += 1
            # didn't find enough anchors, so the function failed.
            if j < len(anchors):
                return POZYX_FAILURE
        return status

    def getTagIds(self, tags, remote_id=None):
        assert len(tags) > 0 and len(
            tags) <= 20, 'getTagIds: size not in range'
        list_size = SingleRegister()
        status = self.getDeviceListSize(list_size, remote_id)
        if list_size[0] < len(tags) or status == POZYX_FAILURE:
            return POZYX_FAILURE
        devices = DeviceList(list_size=list_size)
        status = self.useFunction(
            POZYX_DEVICES_GETIDS, Data([]), devices, remote_id)

        if status == POZYX_SUCCESS:
            for i in range(len(tags)):
                tags[i] = 0x0
            j = 0
            for i in range(list_size[0]):
                data = Data([0] * 3)
                status &= self.useFunction(
                    POZYX_DEVICE_GETINFO, NetworkID(devices[i]), data)
                # why + 1?
                if data[2] == POZYX_TAG_MODE + 1:
                    tags[j] = devices[i]
                    j += 1
            # didn't find enough tags, so the function failed.
            if j < len(tags):
                return POZYX_FAILURE
        return status

    def getDeviceCoordinates(self, device_id, coordinates, remote_id=None):
        if not dataCheck(device_id):
            device_id = NetworkID(device_id)
        assert device_id[0] != 0, 'getDeviceCoordinates: device ID = 0'
        return self.useFunction(
            POZYX_DEVICE_GETCOORDS, device_id, coordinates, remote_id)

    def getDeviceRangeInfo(self, device_id, device_range, remote_id=None):
        if not dataCheck(device_id):
            device_id = NetworkID(device_id)
        assert device_id[0] != 0, 'getDeviceRangeInfo: device ID = 0'
        return self.useFunction(POZYX_DEVICE_GETRANGEINFO, device_id, device_range, remote_id)

    def setInterruptMask(self, mask, remote_id=None):
        if not dataCheck(mask):
            mask = SingleRegister(mask)
        return self.setWrite(POZYX_INT_MASK, mask, remote_id)

    def setUpdateInterval(self, ms, remote_id=None):
        if not dataCheck(ms):
            ms = SingleRegister(ms, size=2)
        assert ms[0] > 100 and ms[
            0] <= 600000, 'setUpdateInterval: ms not 100<ms<60000'
        return self.setWrite(POZYX_POS_INTERVAL, ms, remote_id)

    def setLedConfig(self, config, remote_id=None):
        if not dataCheck(config):
            config = SingleRegister(config)
        return self.setWrite(POZYX_CONFIG_LEDS, config, remote_id)

    def setNetworkId(self, network_id, remote_id=None):
        if not dataCheck(network_id):
            network_id = NetworkID(network_id)
        return self.setWrite(POZYX_NETWORK_ID, network_id, remote_id)

    def setUWBSettings(self, UWB_settings, remote_id=None):
        if not dataCheck(UWB_settings):
            UWB_settings = UWBSettings(UWB_settings[0], UWB_settings[1], UWB_settings[
                                       2], UWB_settings[3], UWB_settings[4])
        gain = Data([UWB_settings.gain_db], 'f')
        UWB = Data([UWB_settings.channel, UWB_settings.bitrate +
                    (UWB_settings.prf >> 6), UWB_settings.plen])
        status = self.setWrite(POZYX_UWB_CHANNEL, UWB, remote_id,
                               2 * POZYX_DELAY_LOCAL_WRITE, 2 * POZYX_DELAY_REMOTE_WRITE)
        if status == POZYX_FAILURE:
            return status
        return self.setTxPower(gain, remote_id)

    def setUWBChannel(self, channel_num, remote_id=None):
        if not dataCheck(channel_num):
            channel_num = SingleRegister(channel_num)
        assert channel_num[0] >= 1 and channel_num[0] <= 7 and channel_num[
            0] != 6, 'setUWBChannel: %i is wrong channel number' % channel_num[0]

        return self.setWrite(POZYX_UWB_CHANNEL, channel_num, remote_id)

    def setTxPower(self, txgain_dB, remote_id=None):
        if not dataCheck(txgain_dB):
            txgain_dB = Data([txgain_dB], 'f')
        assert txgain_dB[0] >= 0.0 and txgain_dB[
            0] <= 35.0, 'setTxPower: TX gain %0.2fdB not in range (0-35dB)' % txgain_dB[0]
        doublegain_dB = Data([int(2.0 * txgain_dB[0] + 0.5)])

        return self.setWrite(POZYX_UWB_GAIN, doublegain_dB, remote_id)

    def setOperationMode(self, mode, remote_id=None):
        if not dataCheck(mode):
            mode = SingleRegister(mode)
        assert mode[0] == POZYX_ANCHOR_MODE or mode[
            0] == POZYX_TAG_MODE, 'setOperationMode: wrong mode'

        return self.setWrite(POZYX_OPERATION_MODE, mode, remote_id)

    def setSensorMode(self, sensor_mode, remote_id=None):
        if not dataCheck(sensor_mode):
            sensor_mode = SingleRegister(sensor_mode)
        assert sensor_mode[0] >= 0 and sensor_mode[
            0] <= 12, 'setSensorMode: mode %i not in range (0-12)' % sensor_mode
        status = self.setWrite(POZYX_INT_MASK, sensor_mode, remote_id)
        # legacy delay?
        time.sleep(POZYX_DELAY_MODE_CHANGE)
        return status

    def setCoordinates(self, coordinates, remote_id=None):
        if not dataCheck(coordinates):
            coordinates = Coordinates(
                coordinates[0], coordinates[1], coordinates[2])
        return self.setWrite(POZYX_POS_X, coordinates, remote_id)

    def setConfigGPIO(self, gpio_num, mode, pull, remote_id=None):
        assert gpio_num > 0 and gpio_num <= 4, 'setConfigGPIO: GPIO number %i not in range' % gpio_num
        assert mode == POZYX_GPIO_DIGITAL_INPUT or mode == POZYX_GPIO_PUSHPULL or mode == POZYX_GPIO_OPENDRAIN, 'setConfigGPIO: wrong mode'
        assert pull == POZYX_GPIO_NOPULL or mode == POZYX_GPIO_PULLUP or mode == POZYX_GPIO_PULLDOWN, 'setConfigGPIO: wrong pull/mode'

        gpio_register = POZYX_CONFIG_GPIO1 + gpio_num - 1
        mask = Data([mode + (pull << 3)])
        return self.setWrite(gpio_register, mask, remote_id)

    def setPositionAlgorithm(self, algorithm, dimension, remote_id=None):
        assert algorithm == POZYX_POS_ALG_UWB_ONLY or algorithm == POZYX_POS_ALG_LS, 'setPositionAlgorithm: wrong algorithm'
        assert dimension == POZYX_3D or dimension == POZYX_2D or dimension == POZYX_2_5D, 'setPositionAlgorithm: wrong dimension'

        params = Data([algorithm + (dimension << 4)])
        return self.setWrite(POZYX_POS_ALG, params, remote_id)

    def setSelectionOfAnchors(self, mode, nr_anchors, remote_id=None):
        assert mode == POZYX_ANCHOR_SEL_MANUAL or mode == POZYX_ANCHOR_SEL_AUTO, 'setSelectionOfAnchors: wrong mode'
        assert nr_anchors > 2 and nr_anchors <= 16, 'setSelectionOfAnchors: num anchors %i not in range 3-16' % nr_anchors

        params = Data([(mode << 7) + nr_anchors])
        return self.setWrite(POZYX_POS_NUM_ANCHORS, params, remote_id)

    def setGPIO(self, gpio_num, value, remote_id=None):
        assert gpio_num >= 1 and gpio_num <= 4, 'setGPIO: GPIO number %i out of bounds' % gpio_num
        if not dataCheck(value):
            value = SingleRegister(value)
        assert value[0] == 0 or value[
            0] == 1, 'setGPIO: wrong value %i' % value

        gpio_register = POZYX_GPIO1 + gpio_num - 1
        return self.setWrite(gpio_register, value, remote_id)

    def setLed(self, led_num, state, remote_id=None):
        assert led_num >= 1 and led_num <= 4, 'setLed: led number %i not in range 1-4' % led_num
        assert state == True or state == False, 'setLed: wrong state'

        params = Data([0x1 << (led_num - 1 + 4) |
                       ((state << led_num - 1) % 256)])
        return self.useFunction(POZYX_LED_CTRL, params, Data([]), remote_id)

    def setPositioningAnchorIds(self, anchors, remote_id=None):
        assert len(anchors) > 0 and len(
            anchors) <= 10, 'setPositioningAnchorIds: anchor_num %i out of range' % len(anchors)

        return self.useFunction(POZYX_POS_SET_ANCHOR_IDS, anchors, Data([]), remote_id)

    # DO functions, like positioning and ranging etc.

    def doRanging(self, destination, device_range, remote_id=None):
        assert destination != 0, 'doRanging: destination can\'t equal zero'
        if not dataCheck(destination):
            destination = NetworkID(destination)

        int_flag = POZYX_INT_STATUS_FUNC
        if remote_id is not None:
            int_flag = POZYX_INT_STATUS_RX_DATA

        status = self.useFunction(
            POZYX_DO_RANGING, destination, Data([]), remote_id)
        if status == POZYX_SUCCESS:
            status = self.checkForFlag(int_flag, POZYX_DELAY_INTERRUPT)
            if status == POZYX_SUCCESS:
                self.getDeviceRangeInfo(destination, device_range, remote_id)
            return status
        return POZYX_FAILURE

    def doPositioning(self, position, dimension=POZYX_2_5D, height=Data([0], 'i'), algorithm=POZYX_POS_ALG_UWB_ONLY, remote_id=None):
        assert algorithm == POZYX_POS_ALG_UWB_ONLY or algorithm == POZYX_POS_ALG_LS, 'doPositioning: wrong algorithm'
        assert dimension == POZYX_3D or dimension == POZYX_2D or dimension == POZYX_2_5D, 'doPositioning: wrong dimension'

        alg_options = Data([dimension << 4 | algorithm])
        status = self.setWrite(POZYX_POS_ALG, alg_options, remote_id)
        if dimension == POZYX_2_5D:
            if not dataCheck(height):
                height = Data([height], 'i')
            status = self.setWrite(POZYX_POS_Z, height, remote_id)

        self.getInterruptStatus(SingleRegister())

        status = self.useFunction(POZYX_DO_POSITIONING, remote_id=remote_id)
        if status != POZYX_SUCCESS:
            return POZYX_FAILURE

        if remote_id is None:
            status = self.checkForFlag(
                POZYX_INT_STATUS_POS, POZYX_DELAY_POSITIONING)
            if status == POZYX_SUCCESS:
                return self.getCoordinates(position)
            return status
        else:
            if self.waitForFlag(POZYX_INT_STATUS_RX_DATA, POZYX_DELAY_REMOTE_POSITIONING) == POZYX_SUCCESS:
                rx_info = Data([0, 0], 'HB')
                self.getRead(POZYX_RX_NETWORK_ID, rx_info)
                if rx_info[0] == remote_id and rx_info[1] == position.byte_size:
                    status = self.readRXBufferData(position)
                    # necessary to update x, y, z variables of position.
                    position.load(position.data)
                    return status
                else:
                    return POZYX_FAILURE
        return POZYX_TIMEOUT

    # performed only remotely.
    def doDiscovery(self, discovery_type=0x0, slots=3, slot_duration=0.01, remote_id=None):
        assert discovery_type == POZYX_DISCOVERY_TAGS_ONLY or discovery_type == POZYX_DISCOVERY_ANCHORS_ONLY or discovery_type == POZYX_DISCOVERY_ALL_DEVICES, 'doDiscovery: wrong type of discovery'
        assert slots > 1 and slots < 10, 'doDiscovery: number of slots %i out of range' % slots
        assert slot_duration > 5, 'doDiscovery: slot duration too short'

        self.getInterruptStatus(SingleRegister())
        params = Data([discovery_type, slots, slot_duration])

        status = self.useFunction(
            POZYX_DEVICES_DISCOVER, params, remote_id=remote_id)
        if status == POZYX_FAILURE:
            return status
        timeout_ms = slot_duration * (slots + 20)
        if remote_id is None:
            return self.checkForFlag(POZYX_INT_STATUS_FUNC, timeout_ms)
        else:
            # give the remote device some time to perform its discovery.
            time.sleep(timeout_ms)
        return status

    def doAnchorCalibration(self, dimension, num_measurements, anchors, heights=None, remote_id=None):
        assert dimension == POZYX_2D or dimension == POZYX_2_5D, 'doAnchorCalibration: wrong dimension'
        assert num_measurements > 0, 'doAnchorCalibration: a negative number of measurements isn\'t allowed'
        assert len(anchors) >= 3 and len(
            anchors) <= 6, 'doAnchorCalibration: num anchors %i out of range 3-6' % len(anchors)
        if not dataCheck(anchors):
            anchors = DeviceList(anchors)
        if dimension == POZYX_2_5D:
            for i in range(len(anchors)):
                anchor_coordinates = Coordinates(0, 0, heights[i])
                anchor = DeviceCoordinates(anchors[i], 0x1, anchor_coordinates)
                self.addDevice(anchor, remote_id)

        self.getInterruptStatus(SingleRegister())
        params = Data([dimension, num_measurements] +
                      anchors.data, 'BB' + anchors.byte_data)
        status = self.useFunction(
            POZYX_DEVICES_CALIBRATE, params, remote_id=remote_id)

        if remote_id is None:
            return self.checkForFlag(POZYX_INT_STATUS_FUNC, 25000)
        else:
            # give the remote device some time to perform calibration
            # has to be thoroughly tested
            time.sleep(POZYX_DELAY_CALIBRATION *
                       len(anchors) * num_measurements / 20)
        return status

    def clearDevices(self, remote_id=None):
        return self.useFunction(POZYX_DEVICES_CLEAR, remote_id=remote_id)

    def addDevice(self, device_coordinates, remote_id=None):
        if not dataCheck(device_coordinates):
            device_coordinates = DeviceCoordinates(device_coordinates[0], device_coordinates[
                1], Coordinates(device_coordinates[2], device_coordinates[3], device_coordinates[4]))

        return self.useFunction(POZYX_DEVICE_ADD, device_coordinates, Data([]), remote_id)

    def resetSystem(self, remote_id=None):
        self.useFunction(POZYX_RESET_SYS, remote_id=remote_id)

    # not implemented, will only be useful with I2C Python.
    def configInterruptPin(self, pin, mode, bActiveHigh, bLatch, remote_id=None):
        pass

    # FLASH functions

    def saveConfiguration(self, save_type, registers, remote_id=None):
        if not dataCheck(registers):
            registers = Data(registers)
        assert save_type == POZYX_FLASH_REGS or save_type == POZYX_FLASH_ANCHOR_IDS or save_type == POZYX_FLASH_NETWORK or save_type == POZYX_FLASH_ALL, 'saveConfiguration: invalid type'
        assert save_type == POZYX_FLASH_REGS or len(
            registers) == 0, 'saveConfiguration: #regs > 0 and not a reg save'
        assert save_type != POZYX_FLASH_REGS or len(
            registers) > 0, 'saveConfiguration: #regs > 0 and not a reg save'

        self.getInterruptStatus(SingleRegister())
        params = Data([save_type] + registers.data)
        status = self.useFunction(
            POZYX_FLASH_SAVE, params, remote_id=remote_id)
        if status == POZYX_FAILURE:
            print("Error saving to flash memory")
            return status
        if remote_id is None:
            return self.checkForFlag(POZYX_INT_STATUS_FUNC, POZYX_DELAY_FLASH)
        else:
            # give the remote device some time to save to flash memory
            time.sleep(POZYX_DELAY_FLASH)
        return status

    def clearConfiguration(self, remote_id=None):
        self.getInterruptStatus(SingleRegister())
        status = self.useFunction(
            POZYX_FLASH_RESET, remote_id=remote_id)
        if status == POZYX_FAILURE:
            print("Error clearing the flash memory")
            return status
        if remote_id is None:
            return self.checkForFlag(POZYX_INT_STATUS_FUNC, POZYX_DELAY_FLASH)
        else:
            # give the remote device some time to clear the flash memory
            time.sleep(POZYX_DELAY_FLASH)
        return status

    def getNumRegistersSaved(self, remote_id=None):
        details = Data([0] * 20)
        if self.useFunction(POZYX_FLASH_DETAILS, data=details, remote_id=remote_id) == POZYX_FAILURE:
            return POZYX_FAILURE

        num = 0
        for i in range(20):
            for j in range(8):
                num += (details[i] >> j) & 0x1
        return num

    def isRegisterSaved(self, regAddress, remote_id=None):
        details = Data([0] * 20)
        if self.useFunction(POZYX_FLASH_DETAILS, data=details, remote_id=remote_id) == POZYX_FAILURE:
            return POZYX_FAILURE
        # no return on bad status here, inconsistent
        byte_num = regAddress / 8
        bit_num = regAddress % 8
        return (details[i] >> bit_num) & 0x1
