import serial, time, threading
from collections import deque
import comm_packet_pb2
import logging
from google.protobuf.message import EncodeError

class SerialCommunication(object):
    def __init__(self, portName, frequency=0.1):
        self.serialPort = serial.Serial(
            port=portName, baudrate=57600, rtscts=True, dsrdtr=True)
        self.NumFailedPackets = 0
    	self.cmdFooter = "SOE!"
    	self.CommFrequency = frequency
        self.MaxPacketFails = 20
        if self.serialPort.isOpen():
            logging.info('Serial communication running on port : ' + portName)
            self.serialPort.flushInput()
            self.serialPort.flushOutput()

    def sendCommand(self, cmd):
    	cmd_and_footer = cmd+self.cmdFooter
        logging.debug(":".join("{:02x}".format(ord(c)) for c in cmd_and_footer))
        self.serialPort.write(cmd_and_footer)

    def commandArduino(self, cmd):
        # Initialize response packet to none
        response = None
        # Keep attempting to command the arduino until we get a response
        # or fail more packets than max packet fails
        while(not response and self.NumFailedPackets <= self.MaxPacketFails):
            # Tx cmds
            self.tx(cmd)
            # Give the Arduino time to respond.
            time.sleep(self.CommFrequency)
            # Rx telemetry
            try:
                response = self.readTelemetry()
            except IOError:
                logging.error("Failed to send single packet :" + str(cmd))
                self.serialPort.flushInput()
                self.serialPort.flushOutput()
                self.NumFailedPackets += 1
        if not response:
            logging.error("Failed to send " + str(self.MaxPacketFails) + " command packets in a row. Check connections.")
            # reset packet fail counter
            self.NumFailedPackets = 0
            raise IOError
        return response

    def tx(self, cmd):
        if (isinstance(cmd, comm_packet_pb2.CommandPacket)):
            # Send down the serialized command
            try:
                self.sendCommand(cmd.SerializeToString())
            except EncodeError:
                logging.error("Failed to encode command packet. Are all required fields set?")
                raise IOError

    def readTelemetry(self):
        bytes_rcvd = self.readRawBytes()
        if bytes_rcvd:
            return self.unpackTelemetry(bytes_rcvd)
        else:
            raise IOError

    def readRawBytes(self):
        bytes_read = ''
        while self.serialPort.inWaiting() > 0:
            bytes_read += self.serialPort.read(1)
        return bytes_read

    def unpackTelemetry(self, raw_bytes):
        wb_tlm = comm_packet_pb2.TelemetryPacket()
        logging.debug("Bytes to be unpacked :")
        logging.debug(":".join("{:02x}".format(ord(c)) for c in raw_bytes))
        try:
            wb_tlm.ParseFromString(raw_bytes)
        except Exception:
            raise IOError
        return wb_tlm

