from pypozyx import *
from pypozyx.definitions.registers import POZYX_TEMPERATURE, POZYX_PRESSURE, POZYX_MAX_LIN_ACC

from time import sleep


# TODO: other structures than the new ones

# CONVERSIONS


def test_conversions(pozyx, remote):
    assert POZYX_ACCEL_DIV_MG == 16.0, "POZYX_ACCEL_DIV_MG wrong, should be 16.0"
    assert POZYX_MAX_LIN_ACCEL_DIV_MG == 1.0, "POZYX_MAX_LIN_ACCEL_DIV_MG wrong, should be 1.0"
    assert POZYX_GYRO_DIV_DPS == 16.0, "POZYX_GYRO_DIV_DPS wrong, should be 16.0"
    assert POZYX_PRESS_DIV_PA == 1000.0, "POZYX_PRESS_DIV_PA wrong, should be 1000.0"
    assert POZYX_TEMP_DIV_CELSIUS == 1.0, "POZYX_TEMP_DIV_CELSIUS wrong, should be 1.0"
    # TODO: more

# TEMPERATURE


def test_temperature_class(pozyx, remote):
    temperature = Temperature()
    other_temperature = Temperature()
    assert temperature == other_temperature, "default temperature objects aren't equal"
    raw_temperature = Data([0], 'b')
    assert raw_temperature == temperature, "temperature not right format of Data([0], 'b')"


def test_raw_temperature_read(pozyx, remote):
    raw_temperature = Data([0], 'b')
    status = pozyx.getRead(POZYX_TEMPERATURE, raw_temperature)
    assert status == POZYX_SUCCESS, "raw read of temperature unsuccessful"
    assert type(raw_temperature[0]) == type(
        1), "temperature value is not an int"


def test_temperature_read(pozyx, remote):
    temperature = Temperature()
    status = pozyx.getTemperature_c(temperature, remote)
    assert status == POZYX_SUCCESS, "getTemperature_c unsuccessful"
    assert type(temperature.value) == type(
        1.0), "temperature value is not a float"
    assert temperature.value > 0, "temperature is freezing"
    assert temperature.value < 150, "temperature is way too hot"
    float_remainder = temperature.value - int(temperature.value)
    assert float_remainder < 0.01, "temperature accuracy not 1 °C"


def test_temperature_conversion(pozyx, remote):
    temperature = Temperature()
    status = pozyx.getTemperature_c(temperature, remote)
    assert status == POZYX_SUCCESS, "getTemperature_c unsuccessful"
    raw_temperature = Data([0], 'b')
    status = pozyx.getRead(POZYX_TEMPERATURE, raw_temperature)
    assert status == POZYX_SUCCESS, "raw read of temperature unsuccessful"
    assert int(temperature.value *
               POZYX_TEMP_DIV_CELSIUS) == raw_temperature[0], "conversion isn't right"


# PRESSURE


def test_pressure_class(pozyx, remote):
    pressure = Pressure()
    other_pressure = Pressure()
    assert pressure == other_pressure, "default pressure objects aren't equal"
    raw_pressure = Data([0], 'I')
    assert raw_pressure == pressure, "pressure not right format of Data([0], 'I')"


def test_raw_pressure_read(pozyx, remote):
    raw_pressure = Data([0], 'I')
    status = pozyx.getRead(POZYX_PRESSURE, raw_pressure)
    assert status == POZYX_SUCCESS, "raw read of pressure unsuccessful"
    assert type(raw_pressure[0]) == type(
        1), "pressure value is not an int"


def test_pressure_read(pozyx, remote):
    pressure = Pressure()
    status = pozyx.getPressure_Pa(pressure, remote)
    assert status == POZYX_SUCCESS, "getPressure_c unsuccessful"
    assert type(pressure.value) == type(
        1.0), "pressure value is not a float"


def test_pressure_conversion(pozyx, remote):
    pressure = Pressure()
    status = pozyx.getPressure_Pa(pressure, remote)
    assert status == POZYX_SUCCESS, "getTemperature_c unsuccessful"
    raw_pressure = Data([0], 'I')
    status = pozyx.getRead(POZYX_PRESSURE, raw_pressure)
    assert status == POZYX_SUCCESS, "raw read of pressure unsuccessful"
    assert int(pressure.value *
               POZYX_PRESS_DIV_PA) == raw_pressure[0], "conversion isn't right"


# MAX_LINEAR_ACCELERATION


def test_max_linear_acceleration_class(pozyx, remote):
    max_linear_acceleration = MaxLinearAcceleration()
    other_max_linear_acceleration = MaxLinearAcceleration()
    assert max_linear_acceleration == other_max_linear_acceleration, "default max_linear_acceleration objects aren't equal"
    raw_max_linear_acceleration = Data([0], 'h')
    assert raw_max_linear_acceleration == max_linear_acceleration, "max_linear_acceleration not right format of Data([0], 'h')"
    sleep(0.01)


def test_raw_max_linear_acceleration_read(pozyx, remote):
    raw_max_linear_acceleration = Data([0], 'h')
    status = pozyx.getRead(POZYX_MAX_LIN_ACC, raw_max_linear_acceleration)
    assert status == POZYX_SUCCESS, "raw read of max_linear_acceleration unsuccessful"
    assert type(raw_max_linear_acceleration[0]) == type(
        1), "max_linear_acceleration value is not an int"
    sleep(0.01)


def test_max_linear_acceleration_read(pozyx, remote):
    max_linear_acceleration = MaxLinearAcceleration()
    status = pozyx.getMaxLinearAcceleration_mg(max_linear_acceleration, remote)
    assert status == POZYX_SUCCESS, "getMaxLinearAcceleration_mg unsuccessful"
    assert type(max_linear_acceleration.value) == type(
        1.0), "max_linear_acceleration value is not a float"
    float_remainder = max_linear_acceleration.value - \
        int(max_linear_acceleration.value)
    assert float_remainder < 0.01, "max_linear_acceleration accuracy not 1 mg"
    sleep(0.01)


def test_max_linear_acceleration_conversion(pozyx, remote):
    raw_max_linear_acceleration = Data([0], 'h')
    status = pozyx.getRead(POZYX_MAX_LIN_ACC, raw_max_linear_acceleration)
    assert status == POZYX_SUCCESS, "raw read of max_linear_acceleration unsuccessful"
    max_linear_acceleration = MaxLinearAcceleration(
        raw_max_linear_acceleration[0])
    assert status == POZYX_SUCCESS, "raw read of max_linear_acceleration unsuccessful"
    assert int(max_linear_acceleration.value *
               POZYX_MAX_LIN_ACCEL_DIV_MG) == raw_max_linear_acceleration[0], "conversion isn't right"


# SENSOR DATA

def test_all_sensor_data(pozyx, remote):
    all_sensor_data = SensorData()
    status = pozyx.getAllSensorData(all_sensor_data)
    assert status == POZYX_SUCCESS, "getAllSensorData not successful"


# ACCELERATION


def test_acceleration(pozyx, remote):
    acceleration = Acceleration()
    status = pozyx.getAcceleration_mg(acceleration)
    assert status == POZYX_SUCCESS, "getAcceleration_mg not successful"
