#!/usr/bin/python
from SerialCommunication import SerialCommunication
from CmdResponseDefinitions import *
import comm_packet_pb2
from cmd import Cmd
from time import sleep
import numpy as np


class RobotTerminal(Cmd):

    def __init__(self):
        Cmd.__init__(self)
        self.portName = "/dev/ttyUSB0"
        self.serialComm = SerialCommunication(self.portName)

    def do_exit(self, args):
        """Exits the terminal"""
        raise SystemExit

    def do_get_active_waypoint(self, args):
        """ Gets the name of the active way point"""
        cmd_packet = comm_packet_pb2.CommandPacket()
        active_waypt_cmd = cmd_packet.RoverCmds.add()
        active_waypt_cmd.Id = WP_GET_ACTIVE
        response = self.arduino_send_command(cmd_packet)
        if response:
            print "The active waypoint is : " + response.ActiveWayPoint
        return

    def do_send_waypoint(self, args):
        """Prompts the user to enter a way-point and then sends it to the Arduino"""
        self.send_waypoint()
        return

    def do_test_drive(self, args):
        """Sends a command to initiate a test drive"""
        cmd_packet = comm_packet_pb2.CommandPacket()
        test_drive_cmd = cmd_packet.RoverCmds.add()
        test_drive_cmd.Id = DO_TEST_DRIVE
        self.arduino_send_command(cmd_packet)

    def do_stop_test_drive(self, args):
        """Sends a command to stop the test drive"""
        cmd_packet = comm_packet_pb2.CommandPacket()
        test_drive_stop_cmd = cmd_packet.RoverCmds.add()
        test_drive_stop_cmd.Id = STOP_TEST_DRIVE
        self.arduino_send_command(cmd_packet)

    def arduino_send_command(self, cmd):
        try:
            response = self.serialComm.commandArduino(cmd)
            print str(response)
            return response
        except (TypeError, IOError):
            print " Communication error, check connections."  
            return None

    def send_waypoint(self):
        try:

            print "WayPoint Name : "
            way_point_name = raw_input()
            print "Heading : "
            way_point_heading = float(raw_input())
            print "Distance : "
            way_point_distance = float(raw_input())
        except ValueError:
            print "Invalid waypoint. Name must be a string less than 15 characters. Heading and distance must be a float."
        way_point_cmd = comm_packet_pb2.CommandPacket()
        way_point_cmd.WayPointCmd.Name = way_point_name
        way_point_cmd.WayPointCmd.Heading = way_point_heading
        way_point_cmd.WayPointCmd.Distance = way_point_distance
        response = self.arduino_send_command(way_point_cmd)
        if self.isValidWayPoint(response):
            print "WayPoint " + way_point_name + " successfully sent and processed."
        else:
            print "WayPoint " + way_point_name + " command was rejected."

    def isValidWayPoint(self, response):
        if response:
            if response.RoverStatus[0].Id == WP_CMD_ACCEPT:
                return True
        return False

if __name__ == '__main__':
    prompt = RobotTerminal()
    prompt.prompt = '>> '
    prompt.cmdloop('Welcome to the Robot Terminal. Type help to see a list of commands.')
