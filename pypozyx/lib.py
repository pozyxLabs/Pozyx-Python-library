#!/usr/bin/env python
"""pypozyx.lib - Contains core and extended Pozyx user functionality through the PozyxLib class."""

from time import sleep
from pypozyx.core import PozyxCore
from pypozyx.definitions import (PozyxBitmasks, PozyxRegisters, PozyxConstants, POZYX_SUCCESS, POZYX_FAILURE,
                                 POZYX_TIMEOUT, ERROR_MESSAGES)
from pypozyx.structures.device import NetworkID, UWBSettings, DeviceList, Coordinates, RXInfo, DeviceCoordinates
from pypozyx.structures.generic import Data, SingleRegister, dataCheck
from pypozyx.structures.sensor_data import PositioningData, RangeInformation

from warnings import warn


class PozyxLib(PozyxCore):
    """PozyxLib

    PozyxLib
    ========

    Implements the functionality users expect from Pozyx, using the methods from PozyxCore
    to communicate and interface with Pozyx both locally and remotely.
    This does not limit itself to positioning, ranging, and reading the sensor data of
    the various Pozyx sensors, but also features an assortment of troubleshooting functions,
    abstractions of frequently used registers, UWB settings, etc.

    Unlike the Arduino library, this isn't divided into parts such as 'device functions',
    'system functions', etc, but will be in the future. For now, the Arduino library should
    work as a great reference.
    """

    # \addtogroup system_functions
    # @{

    def setSensorMode(self, sensor_mode, remote_id=None):
        """Set the Pozyx's sensor mode.

        Args:
            sensor_mode: New sensor mode. See PozyxRegisters.SENSORS_MODE register. integer sensor_mode or SingleRegister(sensor_mode).
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        if not dataCheck(sensor_mode):
            sensor_mode = SingleRegister(sensor_mode)
        assert sensor_mode[0] >= 0 and sensor_mode[
            0] <= 12, 'setSensorMode: mode %i not in range (0-12)' % sensor_mode
        status = self.setWrite(PozyxRegisters.SENSORS_MODE, sensor_mode, remote_id)
        # legacy delay?
        sleep(PozyxConstants.DELAY_MODE_CHANGE)
        return status

    def resetSystem(self, remote_id=None):
        """Resets the Pozyx device.

        Args:
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        return self.useFunction(PozyxRegisters.RESET_SYSTEM, remote_id=remote_id)

    def saveRegisters(self, registers, remote_id=None):
        """Saves the given registers to the Pozyx's flash memory, if these are writable registers.

        This means that upon reset, the Pozyx will use these saved values instead of the default values.
        This is especially practical when changing UWB settings of an entire network, making it unnecessary
        to re - set these when resetting or repowering a device.\n
        DISCLAIMER: Make sure to not abuse this function in your code, as the flash memory only has a finite
        number of writecycles available, adhere to the Arduino's mentality in using flash memory.

        Args:
            registers: Registers to save to the flash memory. Data([register1, register2, ...]) or [register1, register2, ...]
                These registers have to be writable. Saving the UWB gain is currently not working.
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        return self.saveConfiguration(
            PozyxConstants.FLASH_SAVE_REGISTERS, registers, remote_id)

    def getNumRegistersSaved(self, remote_id=None):
        """Obtains the number of registers saved to the Pozyx's flash memory.

        Args:
            remote_id (optional): Remote Pozyx ID.

        Returns:
            The number of saved registers.
        """
        details = Data([0] * 20)
        if self.useFunction(PozyxRegisters.GET_FLASH_DETAILS, data=details, remote_id=remote_id) == POZYX_FAILURE:
            return POZYX_FAILURE

        number_of_saved_registers = 0
        for i in range(20):
            for j in range(8):
                number_of_saved_registers += (details[i] >> j) & 0x1
        return number_of_saved_registers

    def isRegisterSaved(self, register_address, remote_id=None):
        """Returns whether the given register is saved to the Pozyx's flash memory.

        Args:
            register_address: Register address to check if saved
            remote_id (optional): Remote Pozyx ID.

        Returns:
            1 if the register is saved, 0 if it's not.
        """
        details = Data([0] * 20)
        if self.useFunction(PozyxRegisters.GET_FLASH_DETAILS, data=details, remote_id=remote_id) == POZYX_FAILURE:
            return POZYX_FAILURE
        byte_num = int(register_address / 8)
        bit_num = register_address % 8
        return (details[byte_num] >> bit_num) & 0x1

    def setConfigGPIO(self, gpio_num, mode, pull, remote_id=None):
        """Set the Pozyx's selected GPIO pin configuration(mode and pull).

        Args:
            gpio_num: GPIO pin number, 1 to 4.
            mode: GPIO configuration mode. integer mode or SingleRegister(mode)
            pull: GPIO configuration pull. integer pull or SingleRegister(pull)
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        if not dataCheck(mode):
            mode = SingleRegister(mode)
        if not dataCheck(pull):
            pull = SingleRegister(pull)
        assert 1 <= gpio_num <= 4, 'setConfigGPIO: GPIO number %i not in range' % gpio_num
        assert mode[0] in [PozyxConstants.GPIO_DIGITAL_INPUT, PozyxConstants.GPIO_PUSH_PULL,
                           PozyxConstants.GPIO_OPEN_DRAIN], 'setConfigGPIO: wrong mode'
        assert pull[0] in [PozyxConstants.GPIO_NO_PULL, PozyxConstants.GPIO_PULL_UP,
                           PozyxConstants.GPIO_PULL_DOWN], 'setConfigGPIO: wrong pull'

        gpio_register = PozyxRegisters.CONFIG_GPIO_1 + gpio_num - 1
        mask = Data([mode[0] + (pull[0] << 3)])
        return self.setWrite(gpio_register, mask, remote_id)

    def setGPIO(self, gpio_num, value, remote_id=None):
        """Set the Pozyx's selected GPIO pin output.

        Args:
            gpio_num: GPIO pin number, 1 to 4
            value: GPIO output value, either HIGH(1) or LOW(0). Physically, 3.3V or 0V. integer value or SingleRegister(value).
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        if not dataCheck(value):
            value = SingleRegister(value)
        assert 1 <= gpio_num <= 4, 'setGPIO: GPIO number %i out of bounds' % gpio_num
        assert value[0] == 0 or value[
            0] == 1, 'setGPIO: wrong value %i' % value

        gpio_register = PozyxRegisters.GPIO_1 + gpio_num - 1
        return self.setWrite(gpio_register, value, remote_id)

    def setLed(self, led_num, state, remote_id=None):
        """Set the Pozyx's selected LED state.

        Args:
            led_num: LED pin number, 1 to 4
            state: LED output state. Boolean. True = on and False = off, you can use POZYX_LED_ON and POZYX_LED_OFF instead.
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        assert 1 <= led_num <= 4, 'setLed: led number %i not in range 1-4' % led_num
        assert state is True or state is False, 'setLed: wrong state'

        params = Data([0x1 << (led_num - 1 + 4) |
                       ((state << led_num - 1) % 256)])
        return self.useFunction(PozyxRegisters.LED_CONTROL, params, None, remote_id)

    def clearConfiguration(self, remote_id=None):
        """Clears the Pozyx's flash memory.

        Args:
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        self.getInterruptStatus(SingleRegister())
        status = self.useFunction(
            PozyxRegisters.RESET_FLASH_MEMORY, remote_id=remote_id)
        if status == POZYX_FAILURE:
            print("Error clearing the flash memory")
            return status
        # give the device some time to clear the flash memory
        sleep(PozyxConstants.DELAY_FLASH)
        return status

    def configInterruptPin(self, pin=0, mode=0, active_high=False, latch=False, remote_id=None):
        """Configures the interrupt pin via the PozyxRegisters.INTERRUPT_PIN register.

        Args:
            pin (optional): The Pozyx's pin ID. 1 to 4 on anchor, 1 to 6 on tag. 0 means no pin. SingleRegister or integer.
            mode (optional): Push-pull (0) or pull (1). SingleRegister or integer. SingleRegister or integer.
            active_high (optional): Is the interrupt voltage active high or low. Boolean.
            latch (optional): Is the interrupt a short pulse or latch till read? Boolean.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        if dataCheck(pin):
            pin = pin[0]
        if dataCheck(mode):
            mode = mode[0]
        assert 0 <= pin <= 6, 'Error: Pin number %i is incorrect, should be between 0 and 6' % pin
        assert mode == 0 or mode == 1, 'Error: Mode is incorrect, should be 0 (PUSH-PULL) or 1 (PULL).'
        int_config = SingleRegister(pin + (mode << 3) + (active_high << 4) + (latch << 5))
        self.setWrite(PozyxRegisters.INTERRUPT_PIN, int_config, remote_id)

    def getWhoAmI(self, who_am_i, remote_id=None):
        """Obtains the Pozyx's WHO_AM_I.

        Args:
            who_am_i: Container for the read data. SingleRegister or Data([0]).
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        return self.getRead(PozyxRegisters.WHO_AM_I, who_am_i, remote_id)

    def getFirmwareVersion(self, firmware, remote_id=None):
        """Obtains the Pozyx's firmware version.

        Args:
            firmware: Container for the read data. SingleRegister or Data([0]).
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        return self.getRead(PozyxRegisters.FIRMWARE_VERSION, firmware, remote_id)

    def getHardwareVersion(self, hardware, remote_id=None):
        """Obtains the Pozyx's hardware version.

        Args:
            hardware: Container for the read data. SingleRegister or Data([0]).
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        return self.getRead(PozyxRegisters.HARDWARE_VERSION, hardware, remote_id)

    def getSelftest(self, selftest, remote_id=None):
        """Obtains the Pozyx's selftest.

        Args:
            selftest: Container for the read data. SingleRegister or Data([0]).
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        return self.getRead(PozyxRegisters.SELFTEST_RESULT, selftest, remote_id)

    def getErrorCode(self, error_code, remote_id=None):
        """Obtains the Pozyx's error code.

        Args:
            error_code: Container for the read data. SingleRegister or Data([0]).
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        return self.getRead(PozyxRegisters.ERROR_CODE, error_code, remote_id)

    def getCalibrationStatus(self, calibration_status, remote_id=None):
        """Obtains the Pozyx's calibration status.

        Args:
            calibration_status: Container for the read data. SingleRegister or Data([0]).
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        return self.getRead(PozyxRegisters.CALIBRATION_STATUS, calibration_status, remote_id)

    def getInterruptMask(self, mask, remote_id=None):
        """Obtains the Pozyx's interrupt mask.

        Args:
            mask: Container for the read data. SingleRegister or Data([0]).
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        return self.getRead(PozyxRegisters.INTERRUPT_MASK, mask, remote_id)

    def getConfigModeGPIO(self, gpio_num, mode, remote_id=None):
        """Obtain the Pozyx's configuration mode of the selected GPIO pin.

        Args:
            gpio_num: GPIO pin number, 1 to 4.
            mode: Container for the read data. SingleRegister() or Data([0]).
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT

        See Also:
            getGPIO, getConfigPullGPIO
        """
        assert gpio_num > 0 or gpio_num <= 4, 'getConfigModeGPIO: GPIO number incorrect'
        gpio_register = PozyxRegisters.CONFIG_GPIO_1 + gpio_num - 1
        status = self.getRead(gpio_register, mode, remote_id)
        mode[0] &= 0x7
        return status

    def getConfigPullGPIO(self, gpio_num, pull, remote_id=None):
        """Obtain the Pozyx's selected GPIO pin pull.

        Args:
            gpio_num: GPIO pin number, 1 to 4.
            pull: Container for the read data. SingleRegister() or Data([0]).
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT

        See Also:
            getGPIO, getConfigModeGPIO
        """
        assert gpio_num > 0 or gpio_num <= 4, 'getConfigPullGPIO: GPIO number incorrect'
        gpio_register = PozyxRegisters.CONFIG_GPIO_1 + gpio_num - 1
        status = self.getRead(gpio_register, pull, remote_id)
        pull[0] = (pull[0] & 0x18) >> 3
        return status

    def getGPIO(self, gpio_num, value, remote_id=None):
        """Obtain the Pozyx's value of the selected GPIO pin, being either HIGH or LOW(physically 3.3V or 0V).

        Args:
            gpio_num: GPIO pin number, 1 to 4.
            value: Container for the read data. SingleRegister() or Data([0]).
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT

        See Also:
            getConfigPullGPIO, getConfigModeGPIO
        """
        assert gpio_num > 0 or gpio_num <= 4, 'getGPIO: GPIO number incorrect'
        gpio_register = PozyxRegisters.GPIO_1 + gpio_num - 1
        return self.getRead(gpio_register, value, remote_id)

    def getErrorMessage(self, error_code):
        """Returns the system error string for the given error code

        Args:
            error_code: Error code for which to return the error message. int or SingleRegister

        Returns:
            string with error description

        See Also:
            getErrorCode, getSystemError
        """
        if dataCheck(error_code):
            error_code = error_code.value
        return ERROR_MESSAGES.get(error_code, "Unknown error 0x%0.02x" % error_code)

    def getSystemError(self, remote_id=None):
        """Returns the Pozyx's system error string.

        Args:
            remote_id (optional): Remote Pozyx ID.

        Returns:
            string with error description

        See Also:
            getErrorCode, getErrorMessage
        """
        error_code = SingleRegister()
        status = self.getErrorCode(error_code, remote_id)

        if status == POZYX_SUCCESS:
            return self.getErrorMessage(error_code)
        return "Error: couldn't retrieve error from device."

    def setInterruptMask(self, mask, remote_id=None):
        """Set the Pozyx's interrupt mask.

        Args:
            mask: Interrupt mask. See PozyxRegisters.INTERRUPT_MASK register. integer mask or SingleRegister(mask)
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        if not dataCheck(mask):
            mask = SingleRegister(mask)
        return self.setWrite(PozyxRegisters.INTERRUPT_MASK, mask, remote_id)

    def setLedConfig(self, config, remote_id=None):
        """Set the Pozyx's LED configuration.

        Args:
            config: LED configuration. See PozyxRegisters.LED_CONFIGURATION register. integer configuration or SingleRegister(configuration)
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        if not dataCheck(config):
            config = SingleRegister(config)
        return self.setWrite(PozyxRegisters.LED_CONFIGURATION, config, remote_id)

    ## @}

    ## \addtogroup positioning_functions
    # @{

    def saveAnchorIds(self, remote_id=None):
        """Saves the anchor IDs used in positioning to the Pozyx's flash memory.

        This means that upon reset, the Pozyx won't need to be recalibrated before performing positioning.

        Args:
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        self.saveConfiguration(PozyxConstants.FLASH_SAVE_ANCHOR_IDS, remote_id=remote_id)

    def getUpdateInterval(self, ms, remote_id=None):
        """Obtains the Pozyx's update interval.

        Args:
            ms: Container for the read data. SingleRegister or Data([0]).
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        return self.getRead(PozyxRegisters.POSITIONING_INTERVAL, ms, remote_id)

    def getRangingProtocol(self, protocol, remote_id=None):
        """Obtains the Pozyx's ranging protocol

        Args:
            protocol: Container for the read protocol data. SingleRegister or Data([0])
            remote_id (optional): Remote Pozyx ID

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        return self.getRead(PozyxRegisters.RANGING_PROTOCOL, protocol, remote_id)

    def setRangingProtocolFast(self, remote_id=None):
        return self.setRangingProtocol(PozyxConstants.RANGE_PROTOCOL_FAST, remote_id=remote_id)

    def setRangingProtocolPrecision(self, remote_id=None):
        return self.setRangingProtocol(PozyxConstants.RANGE_PROTOCOL_PRECISION, remote_id=remote_id)

    def setRangingProtocol(self, protocol, remote_id=None):
        """Set the Pozyx's ranging protocol.

        Args:
            protocol: the new ranging protocol. See PozyxRegisters.RANGING_PROTOCOL register. integer or SingleRegister(protocol)
            remote_id (optional): Remote Pozyx ID

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        if not dataCheck(protocol):
            protocol = SingleRegister(protocol)
        assert 0 <= protocol[0] <= 2, 'setRangingProtocol: wrong protocol %i' % protocol[0]

        return self.setWrite(PozyxRegisters.RANGING_PROTOCOL, protocol, remote_id)

    def getPositionAlgorithm(self, algorithm, remote_id=None):
        """Obtains the Pozyx's positioning algorithm.

        Args:
            algorithm: Container for the read data. SingleRegister or Data([0]).
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        status = self.getRead(PozyxRegisters.POSITIONING_ALGORITHM, algorithm, remote_id)
        algorithm[0] &= 0xF
        return status

    def getPositionDimension(self, dimension, remote_id=None):
        """Obtains the Pozyx's positioning dimension.

        Args:
            dimension: Container the for read data. SingleRegister or Data([0]).
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        status = self.getRead(PozyxRegisters.POSITIONING_ALGORITHM, dimension, remote_id)
        dimension[0] = (dimension[0] & 0x30) >> 4
        return status

    def getAnchorSelectionMode(self, mode, remote_id=None):
        """Obtains the Pozyx's anchor selection mode.

        Args:
            mode: Container for the read data. SingleRegister or Data([0]).
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        status = self.getRead(PozyxRegisters.POSITIONING_NUMBER_OF_ANCHORS, mode, remote_id)
        mode[0] = (mode[0] & 0x80) >> 7
        return status

    def getNumberOfAnchors(self, nr_anchors, remote_id=None):
        """Obtains the Pozyx's number of selected anchors.

        Args:
            nr_anchors: Container for the read data. SingleRegister or Data([0]).
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        status = self.getRead(PozyxRegisters.POSITIONING_NUMBER_OF_ANCHORS, nr_anchors, remote_id)
        nr_anchors[0] &= 0xF
        return status

    def getOperationMode(self, mode, remote_id=None):
        """Obtains the Pozyx's mode of operation.

        Args:
            mode: Container for the read data. SingleRegister or Data([0]).
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        return self.getRead(PozyxRegisters.OPERATION_MODE, mode, remote_id)

    def getCoordinates(self, coordinates, remote_id=None):
        """Obtains the Pozyx's coordinates. These are either set manually or by positioning.

        Args:
            coordinates: Container for the read data. Coordinates().
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        return self.getRead(PozyxRegisters.POSITION_X, coordinates, remote_id)

    def getHeight(self, height, remote_id=None):
        """Obtains the Pozyx's height coordinate.

        Args:
            height: Container for the read height data. Data([0], 'i').
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        return self.getRead(PozyxRegisters.POSITION_Z, height, remote_id)

    def getPositionError(self, pos_error, remote_id=None):
        """Obtains the Pozyx's positioning error.

        Args:
            pos_error: Container for the read data. PositionError().
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        return self.getRead(PozyxRegisters.POSITIONING_ERROR_X, pos_error, remote_id)

    def getPositioningAnchorIds(self, anchors, remote_id=None):
        """Obtain the IDs of the anchors in the Pozyx's device list used for positioning.

        You need to make sure to know how many anchors are used, as an incorrect
        size of anchors will cause the function to fail. Use getNumberOfAnchors
        to know this number.

        Args:
            anchors: Container for the read data. DeviceList(list_size=size)
            or Data([0] * size, 'H' * size).
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT

        See Also:
            getAnchorIds, getTagIds, getDeviceIds

        Example:
            >> > list_size = SingleRegister()
            >> > self.getNumberOfAnchors(list_size)
            >> > anchor_list = DeviceList(list_size=list_size[0])
            >> > self.getPositioningAnchorIds(anchor_list)
            >> > print(anchor_list)
            '0x6720, 0x6811, 0x6891'
        """
        assert len(anchors) > 0 and len(
            anchors) <= 10, 'getPositioningAnchorIds: Anchor number out of range'
        device_list_size = SingleRegister()
        status = self.getDeviceListSize(device_list_size, remote_id)
        if len(anchors) < device_list_size[0] or status == POZYX_FAILURE:
            return POZYX_FAILURE
        return self.useFunction(PozyxRegisters.GET_POSITIONING_ANCHOR_IDS, Data([]), anchors, remote_id)

    def getDeviceRangeInfo(self, device_id, device_range, remote_id=None):
        """Obtain the range information of the device with selected ID in the Pozyx's device list.

        Args:
            device_id: ID of desired device whose coordinates are of interest. NetworkID()
            or Data([ID], 'H') or integer ID.
            device_range: Container for the read data. DeviceRange().
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        if not dataCheck(device_id):
            device_id = NetworkID(device_id)
        assert device_id[0] != 0, 'getDeviceRangeInfo: device ID = 0'
        return self.useFunction(PozyxRegisters.GET_DEVICE_RANGE_INFO, device_id, device_range, remote_id)

    def setUpdateInterval(self, ms, remote_id=None):
        """Set the Pozyx's update interval in ms(milliseconds).

        Args:
            ms: Update interval in ms. integer ms or SingleRegister(ms, size=2)
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        if not dataCheck(ms):
            ms = SingleRegister(ms, size=2)
        assert ms[0] > 100 and ms[
            0] <= 600000, 'setUpdateInterval: ms not 100<ms<60000'
        return self.setWrite(PozyxRegisters.POSITIONING_INTERVAL, ms, remote_id)

    def setCoordinates(self, coordinates, remote_id=None):
        """Set the Pozyx's coordinates.

        Args:
            coordinates: Desired Pozyx coordinates. Coordinates() or [x, y, z].
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        if not dataCheck(coordinates):
            coordinates = Coordinates(
                coordinates[0], coordinates[1], coordinates[2])
        return self.setWrite(PozyxRegisters.POSITION_X, coordinates, remote_id)

    def setHeight(self, height, remote_id=None):
        """Sets the Pozyx device's height.

        Args:
            height: Desired Pozyx height. integer height or Data([height], 'i').
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        if not dataCheck(height):
            height = Data([height], 'i')
        return self.setWrite(PozyxRegisters.POSITION_Z, height, remote_id=remote_id)

    def setPositioningFilterNone(self, remote_id=None):
        return self.setPositionFilter(PozyxConstants.FILTER_TYPE_NONE, SingleRegister(), remote_id=remote_id)

    def setPositioningFilterFIR(self, filter_strength, remote_id=None):
        return self.setPositionFilter(PozyxConstants.FILTER_TYPE_FIR, filter_strength, remote_id=remote_id)

    def setPositioningFilterMovingMedian(self, filter_strength, remote_id=None):
        return self.setPositionFilter(PozyxConstants.FILTER_TYPE_MOVING_MEDIAN, filter_strength, remote_id=remote_id)

    def setPositioningFilterMovingAverage(self, filter_strength, remote_id=None):
        return self.setPositionFilter(PozyxConstants.FILTER_TYPE_MOVING_AVERAGE, filter_strength, remote_id=remote_id)

    def setPositionFilter(self, filter_type, filter_strength, remote_id=None):
        """Set the Pozyx's positioning filter.

        Note that currently only PozyxConstants.FILTER_TYPE_MOVING_AVERAGE, PozyxConstants.FILTER_TYPE_MOVING_MEDIAN and PozyxConstants.FILTER_TYPE_FIR are implemented.

        Args:
            filter_type: Positioning filter type. Integer or SingleRegister.
            filter_strength: Positioning filter strength. Integer or SingleRegister.
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        if not dataCheck(filter_strength):
            filter_strength = SingleRegister(filter_strength)
        if not dataCheck(filter_type):
            filter_type = SingleRegister(filter_type)
        assert filter_type[0] in [PozyxConstants.FILTER_TYPE_MOVING_AVERAGE, PozyxConstants.FILTER_TYPE_MOVING_MEDIAN,
                                  PozyxConstants.FILTER_TYPE_FIR, PozyxConstants.FILTER_TYPE_NONE], 'setPositionFilter: wrong filter type'
        assert 0 <= filter_strength[0] <= 15, 'setPositionFilter: wrong strength'

        params = Data([filter_type[0] + (filter_strength[0] << 4)])
        return self.setWrite(PozyxRegisters.POSITIONING_FILTER, params, remote_id)

    def setPositionAlgorithmNormal(self, remote_id=None):
        dimension = SingleRegister()
        self.getPositionDimension(dimension, remote_id=remote_id)
        return self.setPositionAlgorithm(PozyxConstants.POSITIONING_ALGORITHM_UWB_ONLY, dimension, remote_id=remote_id)

    def setPositionAlgorithmTracking(self, remote_id=None):
        dimension = SingleRegister()
        self.getPositionDimension(dimension, remote_id=remote_id)
        return self.setPositionAlgorithm(PozyxConstants.POSITIONING_ALGORITHM_TRACKING, dimension, remote_id=remote_id)

    def setPositionAlgorithm(self, algorithm, dimension, remote_id=None):
        """Set the Pozyx's positioning algorithm.

        Note that currently only PozyxConstants.POSITIONING_ALGORITHM_UWB_ONLY and PozyxConstants.POSITIONING_ALGORITHM_TRACKING are implemented.

        Args:
            algorithm: Positioning algorithm. integer algorithm or SingleRegister(algorithm).
            dimension: Positioning dimension. integer dimension or SingleRegister(dimension).
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        if not dataCheck(algorithm):
            algorithm = SingleRegister(algorithm)
        if not dataCheck(dimension):
            dimension = SingleRegister(dimension)
        assert algorithm[0] in [PozyxConstants.POSITIONING_ALGORITHM_UWB_ONLY,
                                PozyxConstants.POSITIONING_ALGORITHM_TRACKING], 'setPositionAlgorithm: wrong algorithm'
        assert dimension[0] in [PozyxConstants.DIMENSION_3D, PozyxConstants.DIMENSION_2D,
                                PozyxConstants.DIMENSION_2_5D], 'setPositionAlgorithm: wrong dimension'

        params = Data([algorithm[0] + (dimension[0] << 4)])
        return self.setWrite(PozyxRegisters.POSITIONING_ALGORITHM, params, remote_id)

    def setSelectionOfAnchorsAutomatic(self, number_of_anchors, remote_id=None):
        return self.setSelectionOfAnchors(PozyxConstants.ANCHOR_SEL_AUTO, number_of_anchors, remote_id=remote_id)

    def setSelectionOfAnchorsManual(self, number_of_anchors, remote_id=None):
        return self.setSelectionOfAnchors(PozyxConstants.ANCHOR_SEL_MANUAL, number_of_anchors, remote_id=remote_id)

    def setSelectionOfAnchors(self, mode, number_of_anchors, remote_id=None):
        """Set the Pozyx's coordinates.

        Args:
            mode: Anchor selection mode. integer mode or SingleRegister(mode).
            number_of_anchors (int, SingleRegister): Number of anchors used in positioning. integer nr_anchors or SingleRegister(nr_anchors).
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        if not dataCheck(mode):
            mode = SingleRegister(mode)
        if not dataCheck(number_of_anchors):
            number_of_anchors = SingleRegister(number_of_anchors)
        assert mode[0] == PozyxConstants.ANCHOR_SEL_MANUAL or mode[
            0] == PozyxConstants.ANCHOR_SEL_AUTO, 'setSelectionOfAnchors: wrong mode'
        assert 2 < number_of_anchors[0] <= 16, 'setSelectionOfAnchors: num anchors %i not in range 3-16' % number_of_anchors[0]

        params = Data([(mode[0] << 7) + number_of_anchors[0]])
        return self.setWrite(PozyxRegisters.POSITIONING_NUMBER_OF_ANCHORS, params, remote_id)

    def setPositioningAnchorIds(self, anchors, remote_id=None):
        """Set the anchors the Pozyx will use for positioning.

        Args:
            anchors: List of anchors that'll be used for positioning. DeviceList() or [anchor_id1, anchor_id2, ...]
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        if not dataCheck(anchors):
            anchors = DeviceList(anchors)
        assert len(anchors) > 0 and len(
            anchors) <= 10, 'setPositioningAnchorIds: anchor_num %i out of range' % len(anchors)

        return self.useFunction(PozyxRegisters.SET_POSITIONING_ANCHOR_IDS, anchors, None, remote_id)

    def doRanging(self, destination, device_range, remote_id=None):
        """Performs ranging with another destination device, resulting in range information.

        This is pretty straightforward, the range information consists of the following:
            - the timestamp of the range measurement.
            - the distance between the local / remote tag and the destination
            - the RSS, which indicates the signal strength between origin and destination.

        While in the Arduino library doRemoteRanging is used for remote ranging, this function
        follows the library's convention to add remote_id as a keyword argument. Make sure that
        the destination is on the same UWB settings as this, and to pass a DeviceRange object
        for the device_range parameter.

        For an in-action example, check the "Ready to range" tutorial on the Pozyx homepage(www.pozyx.io),
        and the ready_to_range.py example found in this library's tutorial folder.

        Args:
            destination: Network ID of the destination, to perform ranging with. integer ID or NetworkID(ID)
            device_range: Container for device range measurement data. DeviceRange object.
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        assert destination != 0, 'doRanging: destination can\'t equal zero'
        if not dataCheck(destination):
            destination = NetworkID(destination)

        int_flag = PozyxBitmasks.INT_STATUS_FUNC
        if remote_id is not None:
            int_flag = PozyxBitmasks.INT_STATUS_RX_DATA

        status = self.useFunction(
            PozyxRegisters.DO_RANGING, destination, Data([]), remote_id)
        if status == POZYX_SUCCESS:
            status = self.checkForFlag(int_flag, PozyxConstants.DELAY_INTERRUPT)
            if status == POZYX_SUCCESS:
                self.getDeviceRangeInfo(destination, device_range, remote_id)
            return status
        return POZYX_FAILURE

    def doPositioning(self, position, dimension=PozyxConstants.DIMENSION_3D, height=Data([0], 'i'), algorithm=None, remote_id=None):
        """Performs positioning with the Pozyx. This is probably why you're using Pozyx.

        This function only performs the positioning and doesn't take care of the previous steps
        required to get this operational, so be sure to adhere to this checklist:
            - while you can perform automatic calibration, manual calibration is much more stable and reliable.
            - when using manual calibration, add all anchors using addDevice.
            - all anchors are on the same UWB settings as the device performing positioning.
            - if you're using more than four anchors, be sure to set this with setSelectionOfAnchors.

        Basic troubleshooting:
            - try to perform ranging with all devices
            - are you using a Coordinates object for your position?
            - if you perform getDeviceListSize and subsequently getDeviceIds, are these your anchors?

        While in the Arduino library doRemotePositioning is used for remote ranging, this function
        follows the library's convention to add remote_id as a keyword argument.

        For an in-action example, check the "Ready to localize" tutorial on the Pozyx homepage(www.pozyx.io),
        and the ready_to_localize.py example found in this library's tutorial folder.

        Args:
            position: Container for the positioning coordinates. Coordinates object.
            dimension (optional): Dimension to perform positioning in. Default 3D. When 2.5D, make sure height is also passed along.
            height (optional): Height of Pozyx in 2.5D positioning. Default 0. Either integer height or Data([height], 'i').
            algorithm (optional): Algorithm set before positioning. No new algorithm is set by default.
            remote_id (optional): Remote Pozyx ID. Local Pozyx is used when None or omitted.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        assert algorithm in [PozyxConstants.POSITIONING_ALGORITHM_UWB_ONLY, PozyxConstants.POSITIONING_ALGORITHM_TRACKING, None], 'doPositioning: wrong algorithm'
        assert dimension in [PozyxConstants.DIMENSION_3D, PozyxConstants.DIMENSION_2D, PozyxConstants.DIMENSION_2_5D], 'doPositioning: wrong dimension'

        if algorithm is not None:
            alg_options = Data([dimension << 4 | algorithm])
            self.setWrite(PozyxRegisters.POSITIONING_ALGORITHM, alg_options, remote_id)
        if dimension == PozyxConstants.DIMENSION_2_5D:
            if not dataCheck(height):
                height = Data([height], 'i')
            self.setWrite(PozyxRegisters.POSITION_Z, height, remote_id)

        firmware_version = SingleRegister()
        self.getFirmwareVersion(firmware_version)
        if firmware_version.value > 0x11:
            position_data = PositioningData(0b1)
            status = self.doPositioningWithData(position_data, remote_id=remote_id)
            if status == POZYX_SUCCESS:
                position.load_bytes(position_data.byte_data)
            return status

        status = self.useFunction(PozyxRegisters.DO_POSITIONING, remote_id=remote_id, params=Data([1], "H"))
        if status != POZYX_SUCCESS:
            return POZYX_FAILURE

        if remote_id is None:
            status = self.checkForFlag(
                PozyxBitmasks.INT_STATUS_POS, PozyxConstants.TIMEOUT_POSITIONING)
            if status == POZYX_SUCCESS:
                return self.getCoordinates(position)
            return status
        else:
            if self.waitForFlag(PozyxBitmasks.INT_STATUS_RX_DATA, PozyxConstants.TIMEOUT_REMOTE_POSITIONING) == POZYX_SUCCESS:
                rx_info = Data([0, 0], 'HB')
                self.getRead(PozyxRegisters.RX_NETWORK_ID, rx_info)
                if rx_info[0] == remote_id and rx_info[1] == position.byte_size:
                    status = self.readRXBufferData(position)
                    # necessary to update x, y, z variables of position.
                    position.load(position.data)
                    return status
                else:
                    return POZYX_FAILURE
        return POZYX_TIMEOUT

    def getPositioningData(self, positioning_data: PositioningData):
        flags = Data([positioning_data.flags], 'H')
        flags.load_hex_string()
        s = 'F,%0.2x,%s,%i\r' % (PozyxRegisters.DO_POSITIONING_WITH_DATA, flags.byte_data, positioning_data.byte_size + 61)
        # very custom solution...
        r = self.serialExchange(s)
        if positioning_data.has_ranges():
            amount_of_ranges = int(r[2 * positioning_data.byte_size:2 * positioning_data.byte_size + 2], 16)
            positioning_data.set_amount_of_ranges(amount_of_ranges)
            r = r[: positioning_data.byte_size * 2 + 2]
        if len(positioning_data) > 0:
            positioning_data.load_bytes(r[2:])
        return int(r[0:2], 16)

    def doPositioningWithData(self, positioning_data, remote_id=None):
        if remote_id is None:
            status = self.useFunction(PozyxRegisters.DO_POSITIONING)

            if status != POZYX_SUCCESS:
                return POZYX_FAILURE
            status = self.checkForFlag(PozyxBitmasks.INT_STATUS_POS, PozyxConstants.TIMEOUT_POSITIONING)
            if status == POZYX_SUCCESS:
                return self.getPositioningData(positioning_data)
            return status
        else:
            flags_data = Data([positioning_data.flags], 'H')
            self.remoteRegFunctionWithoutCheck(remote_id, PozyxRegisters.DO_POSITIONING, flags_data)

            status = self.waitForFlag(PozyxBitmasks.INT_STATUS_RX_DATA, PozyxConstants.TIMEOUT_POSITIONING)
            if status == POZYX_SUCCESS:
                rx_info = RXInfo()
                self.getRxInfo(rx_info)

                if positioning_data.has_ranges():
                    amount_of_ranges = int(
                        (rx_info.amount_of_bytes - positioning_data.byte_size) / RangeInformation.byte_size)
                    positioning_data.set_amount_of_ranges(amount_of_ranges)

                if rx_info.remote_id == remote_id:
                    status = self.readRXBufferData(positioning_data)
                    return status
                else:
                    return POZYX_FAILURE
            return status

    ## @}

    def waitForFlagSafeFast(self, interrupt_flag, timeout_s, interrupt=None):
        from time import time, sleep
        if interrupt is None:
            interrupt = SingleRegister()
        start = time()
        while (time() - start) < timeout_s:
            sleep(PozyxConstants.DELAY_POLLING * 0.33)
            status = self.getInterruptStatus(interrupt)
            if (interrupt[0] & interrupt_flag) and status == POZYX_SUCCESS:
                return True
        return False

    def checkForFlagFast(self, interrupt_flag, timeout_s, interrupt=None):
        if interrupt is None:
            interrupt = SingleRegister()
        error_interrupt_mask = PozyxBitmasks.INT_MASK_ERR
        if self.waitForFlagSafeFast(interrupt_flag | error_interrupt_mask, timeout_s, interrupt):
            if (interrupt[0] & error_interrupt_mask) == error_interrupt_mask:
                return POZYX_FAILURE
            else:
                return POZYX_SUCCESS
        else:
            return PozyxConstants.STATUS_TIMEOUT


    def getReceivedData(self, destination, address, params, data):
        pass

    def remoteRegFunctionOnlyData(self, destination, address, params, data):
        send_data = Data([0, address] + params.data, 'BB' + params.data_format)
        status = self.regFunction(PozyxRegisters.WRITE_TX_DATA, send_data, Data([]))
        if status != POZYX_SUCCESS:
            return status

        self.getInterruptStatus(SingleRegister())
        status = self.sendTXFunction(destination)
        if status != POZYX_SUCCESS:
            return status

        status = self.checkForFlagFast(PozyxBitmasks.INT_STATUS_RX_DATA, 1)
        if status == POZYX_SUCCESS:
            rx_info = RXInfo()
            self.getRxInfo(rx_info)
            if rx_info.remote_id == destination and rx_info.amount_of_bytes == 4:
                return_data = Data([0], 'I')
                self.readRXBufferData(return_data)
                return return_data[0]

        status = self.checkForFlagFast(PozyxBitmasks.INT_STATUS_RX_DATA, 1)
        if status == POZYX_SUCCESS:
            rx_info = RXInfo()
            self.getRxInfo(rx_info)
            if rx_info.remote_id == destination and rx_info.amount_of_bytes == 4:
                return_data = Data([0], 'I')
                self.readRXBufferData(return_data)
                return return_data[0]

    def rangingWithoutCheck(self, destination, device_range, remote_id=None):
        assert destination != 0, "doRanging: destination can't equal zero"
        if not dataCheck(destination):
            destination = NetworkID(destination)

        if remote_id is None:
            return self.doRanging(destination, device_range, remote_id=remote_id)

        distance = self.remoteRegFunctionOnlyData(remote_id, PozyxRegisters.DO_RANGING, destination, Data([]))

        if distance is not None:
            device_range.distance = distance
            return POZYX_SUCCESS
        return POZYX_FAILURE

    ## \addtogroup sensor_data
    # @{

    def getSensorMode(self, sensor_mode, remote_id=None):
        """Obtains the Pozyx's sensor mode.

        Args:
            sensor_mode: Container for the read data. SingleRegister or Data([0]).
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        return self.getRead(PozyxRegisters.SENSORS_MODE , sensor_mode, remote_id)

    def getAllSensorData(self, sensor_data, remote_id=None):
        """Obtains all the Pozyx's sensor data in their default units.

        Args:
            sensor_data: Container for the read data. SensorData() or RawSensorData().
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        return self.getRead(PozyxRegisters.PRESSURE, sensor_data, remote_id)

    def getPressure_Pa(self, pressure, remote_id=None):
        """Obtain the Pozyx's pressure sensor data in Pa(pascal).

        Args:
            pressure: Container for the read data. Pressure or Data([0], 'I') (Data is DEPRECATED).
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        status = self.getRead(PozyxRegisters.PRESSURE, pressure, remote_id)
        if pressure.__class__.__name__ == "Data":
            warn("Using Data instance in getPressure_Pa is deprecated, use Pressure instead",
                 DeprecationWarning)
            pressure[0] = pressure[0] / PozyxConstants.PRESSURE_DIV_PA
        return status

    def getMaxLinearAcceleration_mg(self, max_linear_acceleration, remote_id=None):
        """Obtain the Pozyx's acceleration sensor data in mg.

        Args:
            max_linear_acceleration: Container for the read data. MaxLinearAcceleration.
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        return self.getRead(PozyxRegisters.MAX_LINEAR_ACCELERATION, max_linear_acceleration, remote_id)

    def getAcceleration_mg(self, acceleration, remote_id=None):
        """Obtain the Pozyx's acceleration sensor data in mg.

        Args:
            acceleration: Container for the read data. Acceleration().
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        return self.getRead(PozyxRegisters.ACCELERATION_X, acceleration, remote_id)

    def getMagnetic_uT(self, magnetic, remote_id=None):
        """Obtain the Pozyx's magnetic sensor data in uT(microtesla).

        Args:
            magnetic: Container for the read data. Magnetic().
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        return self.getRead(PozyxRegisters.MAGNETIC_X, magnetic, remote_id)

    def getAngularVelocity_dps(self, angular_vel, remote_id=None):
        """Obtain the Pozyx's angular velocity sensor data in dps(degrees per second).

        Args:
            angular_vel: Container for the read data. AngularVelocity().
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        return self.getRead(PozyxRegisters.GYRO_X, angular_vel, remote_id)

    def getEulerAngles_deg(self, euler_angles, remote_id=None):
        """Obtain the Pozyx's euler angles sensor data in degrees(heading, roll, pitch).

        Args:
            euler_angles: Container for the read data. EulerAngles().
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        return self.getRead(PozyxRegisters.EULER_ANGLE_HEADING, euler_angles, remote_id)

    def getNormalizedQuaternion(self, quaternion, remote_id=None):
        """Obtain the Pozyx's normalized quaternion sensor data that is required for ROS.

        Args:
            quaternion: Container for the read data. Quaternion().
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        status = self.getQuaternion(quaternion, remote_id)
        if status == POZYX_SUCCESS:
            quaternion.normalize()
        return status

    def getQuaternion(self, quaternion, remote_id=None):
        """Obtain the Pozyx's quaternion sensor data.

        Args:
            quaternion: Container for the read data. Quaternion().
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        return self.getRead(PozyxRegisters.QUATERNION_W, quaternion, remote_id)

    def getLinearAcceleration_mg(self, linear_acceleration, remote_id=None):
        """Obtain the Pozyx's linear acceleration sensor data in mg.

        Args:
            linear_acceleration: Container for the read data. LinearAcceleration() or Acceleration().
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        return self.getRead(PozyxRegisters.LINEAR_ACCELERATION_X, linear_acceleration, remote_id)

    def getGravityVector_mg(self, gravity_vector, remote_id=None):
        """Obtain the Pozyx's gravity vector sensor data in mg.

        Args:
            gravity_vector: Container for the read data. Acceleration().
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        return self.getRead(PozyxRegisters.GRAVITY_VECTOR_X, gravity_vector, remote_id)

    def getTemperature_c(self, temperature, remote_id=None):
        """Obtain the Pozyx's temperature sensor data in C(celsius).

        Args:
            temperature: Container for the read data. Temperature or Data([0], 'b') (DEPRECATED).
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        status = self.getRead(
            PozyxRegisters.TEMPERATURE, temperature, remote_id)
        if temperature.__class__.__name__ == "Data" or temperature.__class__.__name__ == "SingleRegister":
            warn("Using Data or SingleRegister instance in getTemperature_c is deprecated, use Temperature instead",
                 DeprecationWarning)
            temperature[0] = temperature[0] / PozyxConstants.TEMPERATURE_DIV_CELSIUS
        return status

    ##  @}

    ## \addtogroup device_list
    # @{

    def getDeviceListSize(self, device_list_size, remote_id=None):
        """Obtain the size of Pozyx's list of added devices.

        Args:
            device_list_size: Container for the read data. SingleRegister() or Data([0]).
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        return self.getRead(PozyxRegisters.DEVICE_LIST_SIZE, device_list_size, remote_id)

    def getDeviceIds(self, devices, remote_id=None):
        """Obtain the IDs of all devices in the Pozyx's device list.

        You need to make sure to know how many devices are in the list, as an incorrect
        size of anchors will cause the function to fail. Use getDeviceListSize
        to know this number.

        Args:
            devices: Container for the read data. DeviceList(list_size=size)
            or Data([0] * size, 'H' * size).
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT

        See Also:
            getAnchorIds, getTagIds, getPositioningAnchorIds

        Example:
            >> > list_size = SingleRegister()
            >> > self.getDeviceListSize(list_size)
            >> > device_list = DeviceList(list_size=list_size[0])
            >> > self.getDeviceIds(device_list)
            >> > print(device_list)
            '0x60A0, 0x6070, 0x6891'
        """
        assert len(devices) > 0 and len(
            devices) <= 20, 'getDeviceIds: size not in range'

        list_size = SingleRegister()

        status = self.getDeviceListSize(list_size, remote_id)
        if list_size[0] < len(devices) or status == POZYX_FAILURE:
            return POZYX_FAILURE

        params = Data([0, list_size[0]])

        return self.useFunction(
            PozyxRegisters.GET_DEVICE_LIST_IDS, params, devices, remote_id)

    def getAnchorIds(self, anchors, remote_id=None):
        """Obtain the IDs of the anchors in the Pozyx's device list.

        You need to make sure to know how many anchors are in the list, as an incorrect
        size of anchors will cause the function to fail.

        Args:
            anchors: Container for the read data. SingleRegister() or Data([0]).
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT

        See Also:
            getDeviceIds, getPositioningAnchorIds, getTagIds
        """
        assert len(anchors) > 0 and len(
            anchors) <= 20, 'getAnchorIds: size not in range'
        list_size = SingleRegister()
        status = self.getDeviceListSize(list_size, remote_id)
        if list_size[0] < len(anchors) or status == POZYX_FAILURE:
            return POZYX_FAILURE
        devices = DeviceList(list_size=list_size)
        status = self.useFunction(
            PozyxRegisters.GET_DEVICE_LIST_IDS, Data([]), devices, remote_id)

        if status == POZYX_SUCCESS:
            for i in range(len(anchors)):
                anchors[i] = 0x0
            j = 0
            for i in range(list_size[0]):
                data = Data([0] * 3)
                status &= self.useFunction(
                    PozyxRegisters.GET_DEVICE_INFO, NetworkID(devices[i]), data)
                # why + 1?
                if data[2] == PozyxConstants.ANCHOR_MODE + 1:
                    anchors[j] = devices[i]
                    j += 1
            # didn't find enough anchors, so the function failed.
            if j < len(anchors):
                return POZYX_FAILURE
        return status

    def getTagIds(self, tags, remote_id=None):
        """Obtain the IDs of the tags in the Pozyx's device list.

        You need to make sure to know how many tags are in the list, as an incorrect
        size of tags will cause the function to fail.

        Args:
            tags: Container for the read data. SingleRegister() or Data([0]).
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT

        See Also:
            getDeviceIds, getAnchorIds, getPositioningAnchorIds
        """
        assert len(tags) > 0 and len(
            tags) <= 20, 'getTagIds: size not in range'
        list_size = SingleRegister()
        status = self.getDeviceListSize(list_size, remote_id)
        if list_size[0] < len(tags) or status == POZYX_FAILURE:
            return POZYX_FAILURE
        devices = DeviceList(list_size=list_size[0])
        status = self.useFunction(
            PozyxRegisters.GET_DEVICE_LIST_IDS, Data([]), devices, remote_id)

        if status == POZYX_SUCCESS:
            for i in range(len(tags)):
                tags[i] = 0x0
            j = 0
            for i in range(list_size[0]):
                data = Data([0] * 3)
                status &= self.useFunction(
                    PozyxRegisters.GET_DEVICE_INFO, NetworkID(devices[i]), data)
                # why + 1?
                if data[2] == PozyxConstants.TAG_MODE + 1:
                    tags[j] = devices[i]
                    j += 1
            # didn't find enough tags, so the function failed.
            if j < len(tags):
                return POZYX_FAILURE
        return status

    def getDeviceCoordinates(self, device_id, coordinates, remote_id=None):
        """Obtain the coordinates of the device with selected ID in the Pozyx's device list.

        Args:
            device_id: ID of desired device whose coordinates are of interest. NetworkID()
            or Data([ID], 'H') or integer ID.
            coordinates: Container for the read data. Coordinates().
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        if not dataCheck(device_id):
            device_id = NetworkID(device_id)
        assert device_id[0] != 0, 'getDeviceCoordinates: device ID = 0'
        return self.useFunction(
            PozyxRegisters.GET_DEVICE_COORDINATES, device_id, coordinates, remote_id)

    def doDiscovery(self, discovery_type=PozyxConstants.DISCOVERY_ANCHORS_ONLY, slots=3, slot_duration=0.01, remote_id=None):
        """Performs discovery on the Pozyx, which will let it discover other Pozyx devices with the same
        UWB settings in range.

        Args:
            discovery_type (optional): Type of devices to discover, defaults to discovering the anchors. PozyxConstants.DISCOVERY_ALL_DEVICES,
                PozyxConstants.DISCOVERY_TAGS_ONLY are alternatives.
            slots (optional): Number of timeslots used in attempt to discover devices. Default is 3 slots.
            slot_duration (optional): Duration in seconds of each timeslot used in the discovery process. Default is 10 ms.
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        assert discovery_type == PozyxConstants.DISCOVERY_TAGS_ONLY or discovery_type == PozyxConstants.DISCOVERY_ANCHORS_ONLY or discovery_type == PozyxConstants.DISCOVERY_ALL_DEVICES, 'doDiscovery: wrong type of discovery'
        assert 1 < slots < 10, 'doDiscovery: number of slots %i out of range' % slots
        assert slot_duration == 0 or int(slot_duration * 1000) > 5, 'doDiscovery: slot duration too short'

        self.getInterruptStatus(SingleRegister())
        params = Data([discovery_type, slots, int(slot_duration * 1000)])

        status = self.useFunction(
            PozyxRegisters.DO_DISCOVERY, params, remote_id=remote_id)
        if status == POZYX_FAILURE:
            return status
        timeout_s = slot_duration * (slots + 20)
        if remote_id is None:
            return self.checkForFlag(PozyxBitmasks.INT_STATUS_FUNC, timeout_s)
        else:
            # give the remote device some time to perform its discovery.
            sleep(timeout_s)
        return status

    def doOptimalDiscovery(self, discovery_type=PozyxConstants.DISCOVERY_ALL_DEVICES, slots=3):
        """Performs a discovery with slot_duration optimised for the device's UWB settings."""
        self.getInterruptStatus(SingleRegister())
        params = Data([discovery_type, slots, 0])
        status = self.useFunction(PozyxRegisters.DO_DISCOVERY, params)
        if status != PozyxConstants.STATUS_SUCCESS:
            return status
        return self.checkForFlag(PozyxBitmasks.INT_STATUS_FUNC, PozyxConstants.TIMEOUT_OPTIMAL_DISCOVERY)

    def doDiscoveryTags(self, slots=3, slot_duration=0.01, remote_id=None):
        """Performs tag discovery on the Pozyx, which will let it discover Pozyx tags with the same
        UWB settings in range.

        Args:
            slots (optional): Number of timeslots used in attempt to discover devices. Default is 3 slots.
            slot_duration (optional): Duration in seconds of each timeslot used in the discovery process. Default is 10 ms.
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        return self.doDiscovery(discovery_type=PozyxConstants.DISCOVERY_TAGS_ONLY, slots=slots,
                                slot_duration=slot_duration, remote_id=remote_id)

    def doDiscoveryAnchors(self, slots=3, slot_duration=0.01, remote_id=None):
        """Performs anchor discovery on the Pozyx, which will let it discover Pozyx anchors with the same
        UWB settings in range.

        Args:
            slots (optional): Number of timeslots used in attempt to discover devices. Default is 3 slots.
            slot_duration (optional): Duration in seconds of each timeslot used in the discovery process. Default is 10 ms.
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        return self.doDiscovery(discovery_type=PozyxConstants.DISCOVERY_ANCHORS_ONLY, slots=slots,
                                slot_duration=slot_duration, remote_id=remote_id)

    def doDiscoveryAll(self, slots=3, slot_duration=0.01, remote_id=None):
        """Performs general discovery on the Pozyx, which will let it discover both Pozyx tags and anchors
        with the same UWB settings in range.

        Args:
            slots (optional): Number of timeslots used in attempt to discover devices. Default is 3 slots.
            slot_duration (optional): Duration in seconds of each timeslot used in the discovery process. Default is 10 ms.
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        return self.doDiscovery(discovery_type=PozyxConstants.DISCOVERY_ALL_DEVICES, slots=slots,
                                slot_duration=slot_duration, remote_id=remote_id)

    def doAnchorCalibration(self, dimension, num_measurements, anchors, heights=None, remote_id=None):
        """Performs automatic anchor calibration on the Pozyx.

        Using manual calibration over automatic calibration is highly recommended, as this will not only
        be less robust to use, the results will also be worse than a carefully accurately manually measured
        setup. Using a laser measurer for this purpose is also adviced.

        When insisting on using automatic calibration, make sure that all devices are in range and able to
        communicate with the device. Try ranging with all devices first, and make sure they're on the same
        UWB settings.

        Args:
            dimension: Dimension for the automatic calibration. When 2.5D, make sure to pass along heights as well.
            num_measurements: Number of measurements to use in calibration. The
            anchors: List of anchor IDs that will be used in the calibration. DeviceList() or [anchor_id1, anchor_id2, ...]
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        warn("Autocalibration will be removed from the library in favor of the much better external functionality in the Pozyx webapp.")
        assert dimension == PozyxConstants.DIMENSION_2D or dimension == PozyxConstants.DIMENSION_2_5D, 'doAnchorCalibration: wrong dimension'
        assert num_measurements > 0, 'doAnchorCalibration: a negative number of measurements isn\'t allowed'
        assert len(anchors) >= 3 and len(
            anchors) <= 6, 'doAnchorCalibration: num anchors %i out of range 3-6' % len(anchors)
        if not dataCheck(anchors):
            anchors = DeviceList(anchors)
        if dimension == PozyxConstants.DIMENSION_2_5D:
            if heights is None:
                heights = [2000] * len(anchors)
            for i in range(len(anchors)):
                anchor_coordinates = Coordinates(0, 0, heights[i])
                anchor = DeviceCoordinates(anchors[i], 0x1, anchor_coordinates)
                self.addDevice(anchor, remote_id)

        self.getInterruptStatus(SingleRegister())
        params = Data([dimension, num_measurements] +
                      anchors.data, 'BB' + anchors.data_format)
        status = self.useFunction(0xC2, params, remote_id=remote_id)

        if remote_id is None:
            return self.checkForFlag(PozyxBitmasks.INT_STATUS_FUNC, 25000)
        else:
            # give the remote device some time to perform calibration
            # has to be thoroughly tested
            sleep(1 * len(anchors) * num_measurements / 20)
        return status

    def clearDevices(self, remote_id=None):
        """Clears the Pozyx's device list.

        Args:
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        return self.useFunction(PozyxRegisters.CLEAR_DEVICES, remote_id=remote_id)

    def addDevice(self, device_coordinates, remote_id=None):
        """Adds a device to the Pozyx's device list. Can be either a tag or anchor.

        Args:
            device_coordinates: Device's ID, flag, and coordinates structure. DeviceCoordinates(ID, flag, Coordinates(x, y, z)) or [ID, flag, x, y, z]
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        if not dataCheck(device_coordinates):
            device_coordinates = DeviceCoordinates(device_coordinates[0], device_coordinates[
                1], Coordinates(device_coordinates[2], device_coordinates[3], device_coordinates[4]))

        return self.useFunction(PozyxRegisters.ADD_DEVICE, device_coordinates, Data([]), remote_id)

    def saveNetwork(self, remote_id=None):
        """Saves the Pozyx's device list to its flash memory.

        This means that upon a reset, the Pozyx will still have the same configured device list.

        Args:
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        status = self.saveConfiguration(PozyxConstants.FLASH_SAVE_NETWORK, remote_id=remote_id)
        status &= self.saveRegisters([PozyxRegisters.POSITIONING_NUMBER_OF_ANCHORS],remote_id=remote_id)
        return status

    def configureAnchors(self, anchor_list, anchor_select=PozyxConstants.ANCHOR_SEL_AUTO, remote_id=None):
        """Configures a set of anchors as the relevant anchors on a device

        Args:
            anchor_list (list): Python list of either DeviceCoordinates or [ID, flag, x, y, z]
            anchor_select (optional): How to select the anchors in positioning
            remote_id (optional): Remote Pozyx ID

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE
        """
        status = POZYX_SUCCESS
        status &= self.clearDevices(remote_id)

        for anchor in anchor_list:
            if not dataCheck(anchor):
                anchor = DeviceCoordinates(
                    anchor[0], anchor[1], Coordinates(anchor[2], anchor[3], anchor[4]))
            if anchor.flag != 0x1:
                warn("ID 0x%0.4x added as tag, is this intentional?" % remote_id)
            status &= self.addDevice(anchor, remote_id)

        if len(anchor_list) < 3 or len(anchor_list) > 16:
            warn("Not enough anchors to do positioning")
            return status

        return status & self.setSelectionOfAnchors(anchor_select, len(anchor_list), remote_id)

    def removeDevice(self, device_id, remote_id=None):
        """Removes a device from the Pozyx's device list, keeping the rest of the list intact

        Args:
            device_id: ID that needs to be removed. NetworkID or integer.
            remote_id (optional): Remote Pozyx ID

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE
        """
        if dataCheck(device_id):
            device_id = device_id[0]

        status = POZYX_SUCCESS
        list_size = SingleRegister()
        status &= self.getDeviceListSize(list_size, remote_id)
        device_list = DeviceList(list_size=list_size[0])
        status &= self.getDeviceIds(device_list, remote_id)
        if device_id not in device_list or status != POZYX_SUCCESS:
            return POZYX_FAILURE
        devices = []
        for id_ in device_list:
            if id_ == device_id:
                continue
            coordinates = Coordinates()
            status = self.getDeviceCoordinates(id_, coordinates, remote_id)
            if status != POZYX_SUCCESS:
                return status
            devices.append(DeviceCoordinates(id_, 0x1, coordinates))

        anchor_select_mode = SingleRegister()

        status = self.getAnchorSelectionMode(anchor_select_mode, remote_id)
        if status != POZYX_SUCCESS:
            return status

        return self.configureAnchors(
            devices, anchor_select=anchor_select_mode, remote_id=remote_id)

    def changeDeviceCoordinates(self, device_id, new_coordinates, remote_id=None):
        """Changes a device's coordinates in the Pozyx's device list, keeping the rest of the list intact

        Args:
            device_id: ID that needs to be removed. NetworkID or integer.
            new_coordinates: new coordinates for the device
            remote_id (optional): Remote Pozyx ID

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE
        """
        if dataCheck(device_id):
            device_id = device_id[0]
        if not dataCheck(new_coordinates):
            new_coordinates = Coordinates(x=new_coordinates[0], y=new_coordinates[
                                          1], z=new_coordinates[2])
        status = POZYX_SUCCESS
        list_size = SingleRegister()
        status &= self.getDeviceListSize(list_size, remote_id)
        device_list = DeviceList(list_size=list_size[0])
        status &= self.getDeviceIds(device_list, remote_id)
        if device_id not in device_list or status is not POZYX_SUCCESS:
            return POZYX_FAILURE
        devices = []
        for id_ in device_list:
            if id_ == device_id:
                devices.append(DeviceCoordinates(id_, 0x1, new_coordinates))
                continue
            coordinates = Coordinates()
            self.getDeviceCoordinates(id_, coordinates, remote_id)
            devices.append(DeviceCoordinates(id_, 0x1, coordinates))

        anchor_select_mode = SingleRegister()
        self.getAnchorSelectionMode(anchor_select_mode, remote_id)

        self.configureAnchors(
            devices, anchor_select=anchor_select_mode, remote_id=remote_id)

    def printDeviceInfo(self, remote_id=None):
        """Prints a Pozyx's basic info, such as firmware.

        Mostly for debugging
        """
        firmware = SingleRegister()
        status = self.getFirmwareVersion(firmware, remote_id)

        print("- Device information")
        if status != POZYX_SUCCESS:
            print("\t- Error: Couldn't retrieve device information")
            return

        print("\t-firmware version %i.%i" %
              (firmware.value >> 4, firmware.value % 0x10))

    def printDeviceList(self, remote_id=None, include_coordinates=True):
        """Prints a Pozyx's device list.

        Args:
            remote_id (optional): Remote Pozyx ID
            include_coordinates (bool, optional): Whether to include coordinates in the prints

        Returns:
            None
        """
        list_size = SingleRegister()
        status = self.getDeviceListSize(list_size, remote_id)

        if list_size[0] == 0:
            print("No devices were found")
            return

        device_list = DeviceList(list_size=list_size[0])
        status &= self.getDeviceIds(device_list, remote_id)

        if status is POZYX_FAILURE:
            print("Couldn't read device list of Pozyx")
            return

        for device_id in device_list:
            if include_coordinates:
                coordinates = Coordinates()
                self.getDeviceCoordinates(device_id, coordinates, remote_id)
                print("\t- %s" % DeviceCoordinates(device_id, 0x1, coordinates))
            else:
                print("\t- 0x%0.4x" % device_id)

    ## @}

    ## \addtogroup communication_functions
    # @{

    def saveUWBSettings(self, remote_id=None):
        """Saves the Pozyx's UWB settings to its flash memory.

        This means that upon a reset, the Pozyx will still have the same configured UWB settings.
        As of writing, PozyxRegisters.UWB_GAIN is not savable yet.

        Args:
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        return self.saveRegisters(PozyxRegisters.ALL_UWB_REGISTERS, remote_id)

    def savePositioningSettings(self, remote_id=None):
        return self.saveRegisters(PozyxRegisters.ALL_POSITIONING_REGISTERS, remote_id)

    def setNetworkId(self, network_id, remote_id=None):
        """Set the Pozyx's network ID.

        If using this remotely, make sure to change the network ID to the new ID in
        subsequent code, as its ID will have changed and using the old ID will not work.

        Args:
            network_id: New Network ID. integer ID or NetworkID(ID) or SingleRegister(ID, size=2)
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        if not dataCheck(network_id):
            network_id = NetworkID(network_id)
        return self.setWrite(PozyxRegisters.NETWORK_ID, network_id, remote_id)

    # TODO find new group for these four functions?
    def remoteRegFunctionWithoutCheck(self, destination, address, params):
        send_data = Data([0, address] + params.data, 'BB' + params.data_format)
        status = self.regFunction(PozyxRegisters.WRITE_TX_DATA, send_data, Data([]))
        if status == POZYX_FAILURE:
            return status

        self.getInterruptStatus(SingleRegister())

        status = self.sendTXFunction(destination)
        if status == POZYX_FAILURE:
            return status

        self.checkForFlag(PozyxBitmasks.INT_STATUS_FUNC, 0.02)

        return status

    def setUWBSettings(self, uwb_settings: UWBSettings, remote_id=None):
        """
        Set the Pozyx's UWB settings.

        If using this remotely, remember to change the local UWB settings as well
        to make sure you are still able to communicate with the remote device.

        Args:
            uwb_settings: The new UWB settings. UWBSettings() or [channel, bitrate, prf, plen, gain_db]

        Kwargs:
            remote_id: Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        if not dataCheck(uwb_settings):
            uwb_settings = UWBSettings(uwb_settings[0], uwb_settings[1],
                                       uwb_settings[2], uwb_settings[3], uwb_settings[4])
        gain = Data([uwb_settings.gain_db], 'f')
        uwb = Data([uwb_settings.channel, uwb_settings.bitrate +
                    (uwb_settings.prf << 6), uwb_settings.plen])

        self.setWrite(PozyxRegisters.UWB_CHANNEL, uwb, remote_id,
                      2 * PozyxConstants.DELAY_LOCAL_WRITE, 2 * PozyxConstants.DELAY_REMOTE_WRITE)

        if remote_id is None:
            return self.setUWBGain(gain, remote_id)
        else:
            return self.doFunctionOnDifferentUWB(self.setUWBGain, uwb_settings, gain, remote_id=remote_id)

    def checkUWBSettings(self, suspected_uwb_settings, remote_id=None, equal_gain=True):
        uwb = UWBSettings()

        if remote_id is None:
            self.getUWBSettings(uwb)
        else:
            self.doFunctionOnDifferentUWB(self.getUWBSettings, suspected_uwb_settings, uwb, remote_id=remote_id)

        if equal_gain is not True:
            uwb.gain_db = suspected_uwb_settings.gain_db
        return uwb == suspected_uwb_settings

    def doFunctionOnDifferentUWB(self, function, uwb_settings, *args, **kwargs):
        original_uwb_settings = UWBSettings()
        self.getUWBSettings(original_uwb_settings)

        self.setUWBSettings(uwb_settings)
        return_value = function(*args, **kwargs)
        self.setUWBSettings(original_uwb_settings)

        return return_value

    def setUWBChannel(self, channel_num, remote_id=None):
        """Set the Pozyx's UWB channel.

        If using this remotely, remember to change the local UWB channel as well
        to make sure you are still able to communicate with the remote device.

        Args:
            channel_num: The new UWB channel, being either 1, 2, 3, 4, 5 or 7.
                See PozyxRegisters.UWB_CHANNEL register. integer channel or SingleRegister(channel)
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        if not dataCheck(channel_num):
            channel_num = SingleRegister(channel_num)
        assert channel_num.value in PozyxConstants.ALL_UWB_CHANNELS, 'setUWBChannel: %i is wrong channel number' % channel_num[0]

        return self.setWrite(PozyxRegisters.UWB_CHANNEL, channel_num, remote_id)

    def setUWBGain(self, uwb_gain_db, remote_id=None):
        """Set the Pozyx's UWB transceiver gain.

        Args:
            uwb_gain_db: The new transceiver gain in dB, a value between 0.0 and 33.0.
                float gain or Data([gain], 'f').
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        if not dataCheck(uwb_gain_db):
            uwb_gain_db = Data([uwb_gain_db], 'f')
        assert uwb_gain_db[0] >= 0.0 and uwb_gain_db[
            0] <= 35.0, 'setUWBGain: TX gain %0.2fdB not in range (0-35dB)' % uwb_gain_db[0]
        doublegain_db = Data([int(2.0 * uwb_gain_db[0] + 0.5)])

        return self.setWrite(PozyxRegisters.UWB_GAIN, doublegain_db, remote_id)

    def setTxPower(self, txgain_db, remote_id=None):
        """DEPRECATED: use getUWBGain instead. Set the Pozyx's UWB transceiver gain.

        Args:
            txgain_db: The new transceiver gain in dB, a value between 0.0 and 33.0.
                float gain or Data([gain], 'f').
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        warn("setTxPower is deprecated, use setUWBGain instead", DeprecationWarning)
        return self.setUWBGain(txgain_db, remote_id)

    def getNetworkId(self, network_id):
        """Obtains the Pozyx's network ID.

        Args:
            network_id: Container for the read data.  NetworkID() or SingleRegister(size=2) or Data([0], 'H').

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE
        """
        return self.regRead(PozyxRegisters.NETWORK_ID, network_id)

    def getUWBSettings(self, UWB_settings, remote_id=None):
        """Obtains the Pozyx's UWB settings.

        Args:
            UWB_settings: Container for the read data.  UWBSettings().
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        # The UWB data register size is 4.
        tmp_data = Data([0] * 4)
        status = self.getRead(PozyxRegisters.UWB_CHANNEL, tmp_data, remote_id)
        UWB_settings.load(tmp_data.data)
        return status

    def getUWBChannel(self, channel_num, remote_id=None):
        """Obtains the Pozyx's UWB channel.

        Args:
            channel_num: Container for the read data. SingleRegister or Data([0]).
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        channel_num[0] = 0
        status = self.getRead(PozyxRegisters.UWB_CHANNEL, channel_num, remote_id)
        if channel_num[0] == 0 or status == POZYX_FAILURE:
            return POZYX_FAILURE
        return status

    def getUWBGain(self, uwb_gain_db, remote_id=None):
        """Obtains the Pozyx's transmitter UWB gain in dB, as a float.

        Args:
            uwb_gain_db: Container for the read data. Data([0], 'f').
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        doublegain_db = SingleRegister()
        status = self.getRead(
            PozyxRegisters.UWB_GAIN, doublegain_db, remote_id)
        uwb_gain_db[0] = 0.5 * doublegain_db[0]
        return status

    def getTxPower(self, txgain_db, remote_id=None):
        """DEPRECATED: use getUWBGain instead. Obtains the Pozyx's transmitter UWB gain in dB, as a float.

        Args:
            txgain_db: Container for the read data. Data([0], 'f').
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        warn("getTxPower is deprecated, use getUWBGain instead", DeprecationWarning)
        return self.getUWBGain(txgain_db, remote_id)

    def getLastNetworkId(self, network_id, remote_id=None):
        """Obtain the network ID of the last device Pozyx communicated with.

        Args:
            network_id: Container for the read data. NetworkID() or SingleRegister(size=2) or Data([0], 'H').
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        return self.getRead(PozyxRegisters.RX_NETWORK_ID, network_id, remote_id)

    def getLastDataLength(self, data_length, remote_id=None):
        """Obtain the size of the most recent data packet received by the Pozyx.

        Args:
            data_length: Container for the read data. SingleRegister() or Data([0]).
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        return self.getRead(PozyxRegisters.RX_DATA_LENGTH, data_length, remote_id)

    ## @}

    def saveConfiguration(self, save_type, registers=None, remote_id=None):
        """General function to save the Pozyx's configuration to its flash memory.

        This constitutes three different Pozyx configurations to save, and each have their specialised derived function:
            POZYX_FLASH_REGS: This saves the passed Pozyx registers if they're writable, see saveRegisters.
            PozyxConstants.FLASH_SAVE_ANCHOR_IDS: This saves the anchors used during positioning, see saveAnchorIds.
            POZYX_FLASH_NETWORK: This saves the device list to the Pozyx device, see saveNetwork.

        It is recommended to use the derived functions, as these are not just easier to use, but also
        more descriptive than this general save function.

        DISCLAIMER: Make sure to not abuse this function in your code, as the flash memory only has a finite
        number of writecycles available, adhere to the Arduino's mentality in using flash memory.

        Args:
            save_type: Type of configuration to save. See above.
            registers (optional): Registers to save to the flash memory. Data([register1, register2, ...]) or [register1, register2, ...]
                These registers have to be writable. Saving the UWB gain is currently not working.
            remote_id (optional): Remote Pozyx ID.

        Returns:
            POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT
        """
        if registers is None:
            registers = Data([])
        if not dataCheck(registers):
            registers = Data(registers)
        assert save_type in [PozyxConstants.FLASH_SAVE_REGISTERS, PozyxConstants.FLASH_SAVE_ANCHOR_IDS,
                             PozyxConstants.FLASH_SAVE_NETWORK], 'saveConfiguration: invalid type'
        assert save_type == PozyxConstants.FLASH_SAVE_REGISTERS or len(
            registers) == 0, 'saveConfiguration: #regs > 0 and not a reg save'
        assert save_type != PozyxConstants.FLASH_SAVE_REGISTERS or len(
            registers) > 0, 'saveConfiguration: #regs > 0 and not a reg save'

        self.getInterruptStatus(SingleRegister())
        params = Data([save_type] + registers.data)
        status = self.useFunction(
            PozyxRegisters.SAVE_FLASH_MEMORY, params, remote_id=remote_id)
        if status == POZYX_FAILURE:
            print("Error saving to flash memory")
            return status
        # give the device some time to save to flash memory
        sleep(PozyxConstants.DELAY_FLASH)
        return status
