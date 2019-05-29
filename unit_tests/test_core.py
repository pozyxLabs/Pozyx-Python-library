from pypozyx import *
from pypozyx.definitions.registers import POZYX_WHO_AM_I, POZYX_CONFIG_GPIO1


def test_read_register(pozyx, remote):
    status = pozyx.getRead(POZYX_WHO_AM_I, SingleRegister(), remote)
    assert status == POZYX_SUCCESS, "register read unsuccessful"


def test_read_5_registers(pozyx, remote):
    status = pozyx.getRead(POZYX_WHO_AM_I, Data(data=[0] * 10), remote)
    assert status == POZYX_SUCCESS, "10 register read unsuccessful"


def test_read_50_registers(pozyx, remote):
    status = pozyx.getRead(POZYX_WHO_AM_I, Data(data=[0] * 50), remote)
    assert status == POZYX_SUCCESS, "50 register read unsuccessful"


def test_read_who_am_i(pozyx, remote):
    who_am_i = SingleRegister()
    status = pozyx.getRead(POZYX_WHO_AM_I, who_am_i, remote)
    assert status == POZYX_SUCCESS, "couldn't read POZYX_WHO_AM_I register"
    assert who_am_i.value == 0x43, "POZYX_WHO_AM_I register has a wrong value"


def test_write_register(pozyx, remote):
    status = pozyx.setWrite(
        POZYX_CONFIG_GPIO1, SingleRegister(0x10), remote)
    assert status == POZYX_SUCCESS, "register write unsuccessful"
    config_gpio1 = SingleRegister()
    status = pozyx.getRead(
        POZYX_CONFIG_GPIO1, config_gpio1, remote) == POZYX_SUCCESS
    assert status == POZYX_SUCCESS, "register read unsuccessful"
    assert config_gpio1.value == 0x10, "written and read values don't match"


def test_no_write_who_am_i(pozyx, remote):
    status = pozyx.setWrite(POZYX_WHO_AM_I, SingleRegister(0x10), remote)
    assert status != POZYX_SUCCESS, "who am i register write unsuccessful"
    who_am_i = SingleRegister()
    status = pozyx.getRead(POZYX_WHO_AM_I, who_am_i, remote)
    assert status == POZYX_SUCCESS, "couldn't read POZYX_WHO_AM_I register"
    assert who_am_i.value != 0x10, "Could write to POZYX_WHO_AM_I"
