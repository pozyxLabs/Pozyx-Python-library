#!/usr/bin/env python
"""
pypozyx.definitions.registers - contains all register definitions used in Pozyx.

It shouldn't be necessary to use these in basic applications as the library functions
should provide a lot of functionality already, but for advanced users looking to
implement their own low-level functionality, these might be very useful.
"""

# Status registers
POZYX_WHO_AM_I = 0x0   # Returns the constant value 0x43.
POZYX_FIRMWARE_VER = 0x1   # Returns the POZYX firmware version.
POZYX_HARDWARE_VER = 0x2   # Returns the POZYX hardware version.
POZYX_ST_RESULT = 0x3   # Returns the self-test result
POZYX_ERRORCODE = 0x4   # Describes a possibly system error.
POZYX_INT_STATUS = 0x5   # Indicates the source of the interrupt.
POZYX_CALIB_STATUS = 0x6   # Returns the calibration status.

# Configuration registers
POZYX_INT_MASK = 0x10    # Indicates which interrupts are enabled.
POZYX_INT_CONFIG = 0x11  # Configure the interrupt pin
POZYX_CONFIG_LEDS = 0x15    # Configure the LEDs
POZYX_POS_FILTER = 0x14    # Filter used for positioning
POZYX_POS_ALG = 0x16    # Algorithm used for positioning
# Configure the number of anchors and selection procedure
POZYX_POS_NUM_ANCHORS = 0x17
# Defines the update interval in ms in continuous positioning.
POZYX_POS_INTERVAL = 0x18
POZYX_NETWORK_ID = 0x1A    # The network id.
POZYX_UWB_CHANNEL = 0x1C    # UWB channel number.
# Configure the UWB datarate and pulse repetition frequency (PRF)
POZYX_UWB_RATES = 0x1D
POZYX_UWB_PLEN = 0x1E    # Configure the UWB preamble length.
POZYX_UWB_GAIN = 0x1F    # Configure the power gain for the UWB transmitter
POZYX_UWB_XTALTRIM = 0x20    # Trimming value for the uwb crystal.
POZYX_RANGE_PROTOCOL = 0x21    # The ranging protocol
# Configure the mode of operation of the pozyx device
POZYX_OPERATION_MODE = 0x22
POZYX_SENSORS_MODE = 0x23    # Configure the mode of operation of the sensors
POZYX_CONFIG_GPIO1 = 0x27    # Configure GPIO pin 1.
POZYX_CONFIG_GPIO2 = 0x28    # Configure GPIO pin 2.
POZYX_CONFIG_GPIO3 = 0x29    # Configure GPIO pin 3.
POZYX_CONFIG_GPIO4 = 0x2A    # Configure GPIO pin 4.

# Positioning data
POZYX_POS_X = 0x30    # x-coordinate of the device in mm.
POZYX_POS_Y = 0x34    # y-coordinate of the device in mm.
POZYX_POS_Z = 0x38    # z-coordinate of the device in mm.
POZYX_POS_ERR_X = 0x3C    # estimated error covariance of x
POZYX_POS_ERR_Y = 0x3E    # estimated error covariance of y
POZYX_POS_ERR_Z = 0x40    # estimated error covariance of z
POZYX_POS_ERR_XY = 0x42    # estimated covariance of xy
POZYX_POS_ERR_XZ = 0x44    # estimated covariance of xz
POZYX_POS_ERR_YZ = 0x46    # estimated covariance of yz

# Sensor data
POZYX_MAX_LIN_ACC = 0x4E  # Return the max linear acceleration and reset it to 0
POZYX_PRESSURE = 0x50    # Pressure data in mPa
POZYX_ACCEL_X = 0x54    # Accelerometer data (in mg)
POZYX_ACCEL_Y = 0x56
POZYX_ACCEL_Z = 0x58
POZYX_MAGN_X = 0x5A    # Magnemtometer data
POZYX_MAGN_Y = 0x5C
POZYX_MAGN_Z = 0x5E
POZYX_GYRO_X = 0x60    # Gyroscope data
POZYX_GYRO_Y = 0x62
POZYX_GYRO_Z = 0x64
# Euler angles heading (or yaw)  (1 degree = 16 LSB )
POZYX_EUL_HEADING = 0x66
POZYX_EUL_ROLL = 0x68    # Euler angles roll ( 1 degree = 16 LSB )
POZYX_EUL_PITCH = 0x6A    # Euler angles pitch ( 1 degree = 16 LSB )
POZYX_QUAT_W = 0x6C    # Weight of quaternion.
POZYX_QUAT_X = 0x6E    # x of quaternion
POZYX_QUAT_Y = 0x70    # y of quaternion
POZYX_QUAT_Z = 0x72    # z of quaternion
POZYX_LIA_X = 0x74    # Linear acceleration in x-direction
POZYX_LIA_Y = 0x76    # Linear acceleration in y-direction
POZYX_LIA_Z = 0x78    # Linear acceleration in z-direction
POZYX_GRAV_X = 0x7A    # x-component of gravity vector
POZYX_GRAV_Y = 0x7C    # y-component of gravity vector
POZYX_GRAV_Z = 0x7E    # z-component of gravity vector
POZYX_TEMPERATURE = 0x80    # Temperature

# General data
# Returns the number of devices stored internally
POZYX_DEVICE_LIST_SIZE = 0x81
POZYX_RX_NETWORK_ID = 0x82    # The network id of the latest received message
POZYX_RX_DATA_LEN = 0x84    # The length of the latest received message
POZYX_GPIO1 = 0x85    # Value of the GPIO pin 1
POZYX_GPIO2 = 0x86    # Value of the GPIO pin 2
POZYX_GPIO3 = 0x87    # Value of the GPIO pin 3
POZYX_GPIO4 = 0x88    # Value of the GPIO pin 4

# Functions
POZYX_RESET_SYS = 0xB0    # Reset the Pozyx device
POZYX_LED_CTRL = 0xB1    # Control LEDS 1 to 4 on the board
POZYX_TX_DATA = 0xB2    # Write data in the UWB transmit (TX) buffer
POZYX_TX_SEND = 0xB3    # Transmit the TX buffer to some other pozyx device
POZYX_RX_DATA = 0xB4    # Read data from the UWB receive (RX) buffer
POZYX_DO_RANGING = 0xB5    # Initiate ranging measurement
POZYX_DO_POSITIONING = 0xB6    # Initiate the positioning process.
# Set the list of anchor ID's used for positioning.
POZYX_POS_SET_ANCHOR_IDS = 0xB7
# Read the list of anchor ID's used for positioning.
POZYX_POS_GET_ANCHOR_IDS = 0xB8
POZYX_FLASH_RESET = 0xB9  # Reset a portion of the configuration in flash memory
POZYX_FLASH_SAVE = 0xBA  # Store a portion of the configuration in flash memory
POZYX_FLASH_DETAILS = 0xBB  # Return information on what is stored in flash

# Device list functions
# Get all the network IDs's of devices in the device list.
POZYX_DEVICES_GETIDS = 0xC0
# Obtain the network ID's of all pozyx devices within range.
POZYX_DEVICES_DISCOVER = 0xC1
# Obtain the coordinates of the pozyx (anchor) devices within range.
POZYX_DEVICES_CALIBRATE = 0xC2
POZYX_DEVICES_CLEAR = 0xC3    # Clear the list of all pozyx devices.
POZYX_DEVICE_ADD = 0xC4    # Add a pozyx device to the devices list
# Get the stored device information for a given pozyx device
POZYX_DEVICE_GETINFO = 0xC5
# Get the stored coordinates of a given pozyx device
POZYX_DEVICE_GETCOORDS = 0xC6
# Get the stored range inforamation of a given pozyx device
POZYX_DEVICE_GETRANGEINFO = 0xC7
POZYX_CIR_DATA = 0xC8    # Get the channel impulse response (CIR) coefficients
