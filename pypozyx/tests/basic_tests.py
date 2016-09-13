# A first test fixture trial.

from pypozyx import *

import unittest

port = '/dev/ttyACM0'


class PozyxSerialCoreTest(unittest.TestCase):

    def setUp(self):
        self.pozyx = PozyxSerial(port)

    def tearDown(self):
        self.pozyx.ser.close()

    def test_who_am_i(self):
        whoami = Data([0])
        self.pozyx.getWhoAmI(whoami)

        # WhoAmI register should always return 0x43
        self.assertEqual(0x43, whoami[0], msg="WhoAmI!= 0x43")
        #print("WhoAmI: %0.2x" % whoami[0])

    def test_network_id(self):
        network_id = NetworkID(0)
        self.pozyx.getNetworkId(network_id)

        self.assertLess(0, network_id[0])
        print("Network ID: ", network_id)


if __name__ == "__main__":
    unittest.main(warnings='ignore')
