import serial, time, threading
from collections import deque
import comm_packet_pb2
import logging
from google.protobuf.message import EncodeError
from CmdResponseDefinitions import *

class SerialCommunication(object):
    def __init__(self, portName, frequency=0.05):
        self.serialPort = serial.Serial(
            port=portName, baudrate=57600, rtscts=True, dsrdtr=True)
    	self.CmdFooter = "SOE!"
    	self.CommFrequency = frequency
        if self.serialPort.isOpen():
            logging.info('Serial communication running on port : ' + portName)
            self.serialPort.flushInput()
            self.serialPort.flushOutput()
    
    def sendWayPoint(self, name, heading, distance):
        max_cmd_attempts = 190
        # Build waypoint command packet 
        way_point_cmd = comm_packet_pb2.CommandPacket()
        way_point_cmd.WayPointCmd.Name = name
        way_point_cmd.WayPointCmd.Heading = heading
        way_point_cmd.WayPointCmd.Distance = distance
        try:
            self.commandArduino(way_point_cmd, max_cmd_attempts)
            print "Waypoint command : " + str(name) + " accepted."
        except IOError:
            print "Waypoint command : " + str(name) + " failed."

    def getActiveWayPointName(self):
        max_cmd_attempts = 20
        cmd_packet = comm_packet_pb2.CommandPacket()
        active_waypt_cmd = cmd_packet.RoverCmds.add()
        active_waypt_cmd.Id = WP_GET_ACTIVE
        try:
            active_waypoint_name = self.commandArduino(cmd_packet, max_cmd_attempts).ActiveWayPoint
            return active_waypoint_name
        except IOError:
            print "Get active waypoint command failed."
        return ""

    def commandTestDrive(self):
        max_cmd_attempts = 20
        cmd_packet = comm_packet_pb2.CommandPacket()
        test_drive_cmd = cmd_packet.RoverCmds.add()
        test_drive_cmd.Id = DO_TEST_DRIVE
        try:
            self.commandArduino(cmd_packet, max_cmd_attempts)
            print "Test drive command accepted."
        except IOError:
            print "Test drive command failed."

    def stopTestDrive(self):
        max_cmd_attempts = 20
        cmd_packet = comm_packet_pb2.CommandPacket()
        test_drive_cmd = cmd_packet.RoverCmds.add()
        test_drive_cmd.Id = STOP_TEST_DRIVE
        try:
            self.commandArduino(cmd_packet, max_cmd_attempts)
            print "Stop test drive command accepted."
        except IOError:
            print "Stop test drive command failed."

    def commandArduino(self, cmd, attemptsUntilTimeout):
        # Initialize response packet to none
        responseOk = False
        num_failed_packets = 0
        # Keep attempting to send a command until we get a command
        # accepted or rejected response
        while(not responseOk and num_failed_packets < attemptsUntilTimeout):
            # Tx cmds
            self.tx(cmd)
            # Give the Arduino time to respond.
            time.sleep(self.CommFrequency)
            # Rx telemetry
            try:
                response = self.readTelemetry()
                if response and len(response.RoverStatus) > 0:
                    responseOk = (response.RoverStatus[0].Id == CMD_ACCEPT or response.RoverStatus[0].Id == CMD_REJECT)
                else:
                    responseOk = False
            except IOError:
                logging.info("Failed to send single packet :" + str(cmd))
                #self.serialPort.flushInput()
                #self.serialPort.flushOutput()
                num_failed_packets += 1
                time.sleep(self.CommFrequency)
        if not responseOk:
            logging.error("Failed to send " + str(attemptsUntilTimeout) + " command packets in a row. Check connections.")
            raise IOError
        if responseOk and (response.RoverStatus[0].Id == CMD_REJECT):
            logging.error("Command " + str(cmd) + " was rejected.")
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

    def sendCommand(self, cmd):
        cmd_and_footer = cmd+self.CmdFooter
        logging.debug(":".join("{:02x}".format(ord(c)) for c in cmd_and_footer))
        self.serialPort.write(cmd_and_footer)

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

