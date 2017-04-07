import pytest
from pypozyx import *

def pytest_addoption(parser):
    parser.addoption("--dec", action="store", default=False,
                     help="whether the device ID is a decimal instead of hex")
    parser.addoption("--remote", action="store", default=None,
                     help="the remote ID for testing when applicable")
    parser.addoption("--interface", action="store", default=None,
                     help="the interface to test, defaults to serial")
    # TODO: add port for serial


@pytest.fixture(scope='session')
def remote(request):
    remote = request.config.getoption("--remote")
    base = 16
    if request.config.getoption("--dec") != False and request.config.getoption("--dec") != '0' and request.config.getoption("--dec") != 'false':
        base = 10
    if remote is not None:
        remote = int(remote, base)
    return remote


@pytest.fixture(scope='session')
def pozyx(request):
    interface = request.config.getoption("--interface")
    return PozyxSerial(get_serial_ports()[0].device)
    if interface == 'serial' or interface is None:
        return PozyxSerial(get_serial_ports()[0].device)
    else:
        print("not a valid setting for interface option, better not use it right now... giving a pozyx serial anyway")
        return PozyxSerial(get_serial_ports()[0].device)
