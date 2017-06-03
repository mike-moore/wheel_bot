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
        self.SuccessTransmissionCount = 0
        self.FailTransmissionCount = 0

    def test_sendCommands(self):
    	for num in range(1, 101):
        	#logging.info("Sending get active waypoint command")
        	cmd_packet = comm_packet_pb2.CommandPacket()
        	control_signal_cmd = cmd_packet.RoverCmds.add()
        	control_signal_cmd.Id = WP_GET_ACTIVE
        	response = self.helper_SendOneCmdPacket(cmd_packet)
        	self.helper_checkResponse(response)

    def helper_SendOneCmdPacket(self, cmd_packet):
        try:
            response = self.testArticle.commandArduino(cmd_packet)
            return response
        except IOError:
            # Fail the test. No response from Arduino
            self.assertTrue(False)
            return None

    def helper_checkResponse(self, response):
        if response:
            self.SuccessTransmissionCount += 1
            logging.info("Success Packet # : " + str(self.SuccessTransmissionCount))
            #logging.info("Dumping received packet : \n" + str(response))
            #self.assertIsInstance(response, comm_packet_pb2.TelemetryPacket)
        else:
            self.FailTransmissionCount += 1
            logging.info("Failed Packet # : " + str(self.FailTransmissionCount))

if __name__ == '__main__':
    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG, format='%(levelname)s:%(message)s')
    # Run the unit-tests
    suite = unittest.TestLoader().loadTestsFromTestCase(CommCheckout)
    unittest.TextTestRunner(verbosity=2).run(suite)