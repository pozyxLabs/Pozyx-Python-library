from pypozyx import *
from pypozyx.definitions.registers import POZYX_WHO_AM_I, POZYX_CONFIG_LEDS

# read tests
if True:
    def test_read_register(pozyx, remote):
        status = pozyx.getRead(0, SingleRegister(), remote)
        assert status == POZYX_SUCCESS

    # TODO move as it's not essential function
    def test_get_who_am_i(pozyx, remote):
        # TODO: Make this for all gets and use pypeazyx
        whoami = SingleRegister()
        status = pozyx.getRead(POZYX_WHO_AM_I, whoami, remote)
        assert whoami.value == 0x43

    # TODO check if also occurs for remote
    def test_all_reads(pozyx):
        can_not_read_negatives = []
        can_read_negatives = []
        for address in range(256):
            print(address)
            single_register = SingleRegister(-1)
            status = pozyx.getRead(address, single_register)
            if is_reg_readable(address):
                if not (status == POZYX_SUCCESS and single_register[0] != -1):
                    can_read_negatives.append(address)
            else:
                if not (status == POZYX_FAILURE and single_register[0] == -1):
                    can_not_read_negatives.append(address)
        assert can_not_read_negatives == []
        assert can_read_negatives == []

# write tests
if False:
    def test_write_register(pozyx, remote):
        single_register = SingleRegister(0)
        status = pozyx.setWrite(POZYX_CONFIG_LEDS, single_register, remote)
        assert status == POZYX_SUCCESS
        single_register = SingleRegister(100)
        status = pozyx.getRead(POZYX_CONFIG_LEDS, single_register)
        assert status == POZYX_SUCCESS
        assert single_register[0] == 0


# function tests
if True:
    pass
