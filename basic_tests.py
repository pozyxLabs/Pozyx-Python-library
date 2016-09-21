# A first test fixture trial.

from pypozyx import *

import unittest

port = '/dev/ttyACM0'

# make separate Core tests


class PozyxSerialCoreTest(unittest.TestCase):

    def setUp(self):
        self.pozyx = PozyxSerial(port)

    def tearDown(self):
        self.pozyx.ser.close()

    def test_who_am_i(self):
        # ridiculous test as this is done automatically when initializing
        # PozyxSerial.
        whoami = Data([0])
        self.pozyx.getWhoAmI(whoami)

        # WhoAmI register should always return 0x43
        self.assertEqual(0x43, whoami[0], msg="WhoAmI!= 0x43")


if __name__ == "__main__":
    unittest.main(warnings='ignore')
