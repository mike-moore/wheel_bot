#!/usr/bin/python
from SerialCommunication import SerialCommunication
import comm_packet_pb2
import sys, logging
from cmd import Cmd
from time import sleep
import csv

class RobotTerminal(Cmd):

    def __init__(self):
        Cmd.__init__(self)
        self.portName = "/dev/ttyUSB1"
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

    def do_send_csv_waypoints(self, args):
        """Opens and reads a set of way-points from a csv file and sends them to the Arduino"""
        self.send_csv_waypoints()
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
            print "   Communication error, check connections.\n"  
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
        self.arduino_send_command(way_point_cmd)

    def send_csv_waypoints(self):
        with open('./Waypoints.csv') as waypoints:
            waypoint_reader = csv.DictReader(waypoints)
            for row in waypoint_reader:
                way_point_name = row['name']
                way_point_heading = float(row['heading'])
                way_point_distance = float(row['distance'])
                try:
                    print "WayPoint Name : ", way_point_name
                    print "Heading : ", way_point_heading
                    print "Distance : ", way_point_distance
                    way_point_cmd = comm_packet_pb2.CommandPacket()
                    way_point_cmd.WayPointCmd.Name = way_point_name
                    way_point_cmd.WayPointCmd.Heading = way_point_heading
                    way_point_cmd.WayPointCmd.Distance = way_point_distance
                    self.arduino_send_command(way_point_cmd)
                    sleep(1.0)
                except ValueError:
                    print "Invalid waypoint. Name must be a string less than 15 characters. Heading and distance must be a float."

if __name__ == '__main__':
    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG, format='%(levelname)s:%(message)s')
    prompt = RobotTerminal()
    prompt.prompt = '>> '
    prompt.cmdloop('Welcome to the Robot Terminal. Type "help" to see a list of commands.')