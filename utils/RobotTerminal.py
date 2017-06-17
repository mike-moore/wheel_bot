#!/usr/bin/python
from SerialCommunication import SerialCommunication
from CmdResponseDefinitions import *
import comm_packet_pb2
import sys, logging
from cmd import Cmd
from time import sleep
import csv

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
        try:
            active_way_point_name = self.serialComm.getActiveWayPointName()
            print "The active waypoint is : " + active_way_point_name
        except IOError:
            logging.error("Failed to get waypoint name.")

    def do_send_waypoint(self, args):
        """Prompts the user to enter a way-point and then sends it to the Arduino"""
        self.send_waypoint()
        return

    def do_send_csv_waypoints(self, args):
        """Opens and reads a set of way-points from a csv file and sends them to the Arduino"""
        self.send_csv_waypoints()
        return

    def do_test_drive(self, args):
        """Sends a command to initiate a test drive"""
        self.serialComm.commandTestDrive()

    def do_stop_test_drive(self, args):
        """Sends a command to stop the test drive"""
        self.serialComm.stopTestDrive()

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
        self.serialComm.sendWayPoint(way_point_name, way_point_heading, way_point_distance)

    def send_csv_waypoints(self):
        with open('./Waypoints.csv') as waypoints:
            waypoint_reader = csv.DictReader(waypoints)
            for row in waypoint_reader:
                way_point_name = row['name']
                way_point_heading = float(row['heading'])
                way_point_distance = float(row['distance'])
                self.serialComm.sendWayPoint(way_point_name, way_point_heading, way_point_distance)

if __name__ == '__main__':
    logging.basicConfig(stream=sys.stderr, level=logging.ERROR, format='%(levelname)s:%(message)s')
    prompt = RobotTerminal()
    prompt.prompt = '>> '
    prompt.cmdloop('Welcome to the Robot Terminal. Type "help" to see a list of commands.')