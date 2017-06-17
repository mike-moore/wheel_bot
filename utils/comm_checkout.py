#!/usr/bin/python

import unittest
import logging, sys
from time import sleep

# Need a serial communication component for this test
from SerialCommunication import SerialCommunication
from CmdResponseDefinitions import *
import comm_packet_pb2


class CommCheckout(unittest.TestCase):

    def setUp(self):
        self.testArticle = SerialCommunication("/dev/ttyUSB0")

    def test_sendCommands(self):
        try:
            for num in range(1, 101):
                print self.testArticle.getActiveWayPointName()
        except IOError:
            self.assertTrue(False)
        self.assertTrue(True)

if __name__ == '__main__':
    logging.basicConfig(stream=sys.stderr, level=logging.ERROR, format='%(levelname)s:%(message)s')
    # Run the unit-tests
    suite = unittest.TestLoader().loadTestsFromTestCase(CommCheckout)
    unittest.TextTestRunner(verbosity=2).run(suite)