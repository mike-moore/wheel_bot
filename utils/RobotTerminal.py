#!/usr/bin/python
from SerialCommunication import SerialCommunication
from CmdResponseDefinitions import *
import comm_packet_pb2
import sys, logging
from cmd import Cmd
from time import sleep
import csv
import numpy as np
import matplotlib.pyplot as plt
import scipy
import scipy.fftpack
import xbox

class RobotTerminal(Cmd):

    def __init__(self):
        Cmd.__init__(self)
        self.portName = "/dev/ttyUSB0"
        self.serialComm = SerialCommunication(self.portName)

    def do_exit(self, args):
        """Exits the terminal"""
        raise SystemExit

    def do_connect_controller(self, args):
        """Connects the xbox controller"""
        xboxController = xbox.Joystick()
        sleep(1)
        self.serialComm.commandManualDrive()
        print "###################################################"
        print "# WheelBot Controls "
        print "###################################################"
        print "Exit/Stop Control           :        Back button"
        print "Drive forward               :        Right Trigger"
        print "Drive backward              :        Left Trigger"
        print "Drive left/right            :        Left Joystick"
        print "###################################################"
        # Loop until back button is pressed
        while not xboxController.Back():
            # Show connection status
            if xboxController.connected():
                print "Connected   "
            else:
                print "Disconnected"
                break
            print "Left/Right value : " + str(xboxController.leftX())
            print "Drive Fwd Throttle : " + str(xboxController.rightTrigger())
            print "Drive Bwd Throttle : " + str(xboxController.leftTrigger())
            self.serialComm.commandMotorRpms(xboxController.rightTrigger()*30.0, xboxController.rightTrigger()*30.0)
            self.serialComm.commandMotorRpms(-xboxController.leftTrigger()*30.0, -xboxController.leftTrigger()*30.0)
            #sleep(0.1)
        print "###################################################"
        print "# Stopping manual control. Exiting...."
        print "###################################################"
        self.serialComm.stopManualDrive()
        xboxController.close()

    def do_heading_filter_analysis(self, args):
        """ Gets the heading"""
        data = np.genfromtxt(args, delimiter=',', names=['heading'])
        data = data['heading'][75:]
        filtered_data_3 = self.moving_average(data, n=3)
        filtered_data_10 = self.moving_average(data, n=10)

        fig1 = plt.figure(figsize=(20, 10), dpi=80, facecolor='w', edgecolor='k')
        fig1_ax1 = fig1.add_subplot(211)
        fig1_ax1.title.set_text('Magnetometer Filters')
        fig1_ax1.set_xlabel('Time (seconds)')
        fig1_ax1.set_ylabel(r'heading   $( r ) $')
        line1, = fig1_ax1.plot(data, label="raw")
        line2, = fig1_ax1.plot(filtered_data_3, label="3 pt avg")
        fig1_ax1.legend(loc='lower right')

        fig1_ax2 = fig1.add_subplot(212)
        fig1_ax2.title.set_text('Magnetometer Filters')
        fig1_ax2.set_xlabel('Time (seconds)')
        fig1_ax2.set_ylabel(r'heading   $( r ) $')
        line1, = fig1_ax2.plot(data, label="raw")
        line2, = fig1_ax2.plot(filtered_data_10, label="10 pt avg")
        fig1_ax2.legend(loc='lower right')

        plt.show()

    def moving_average(self, a, n=3) :
        ret = np.cumsum(a, dtype=float)
        ret[n:] = ret[n:] - ret[:-n]
        return ret[n - 1:] / n

    def do_plot_heading(self, args):
        """ Gets the heading"""
        data = np.genfromtxt(args, delimiter=',', names=['heading'])
        data = data[75:]
        fft = abs(scipy.fft(data))
        freqs = scipy.fftpack.fftfreq(data.size, 0.01)

        fig1 = plt.figure(figsize=(20, 10), dpi=80, facecolor='w', edgecolor='k')
        fig1_ax1 = fig1.add_subplot(211)
        fig1_ax1.title.set_text('Magnetometer Data')
        fig1_ax1.set_xlabel('Time (seconds)')
        fig1_ax1.set_ylabel(r'heading   $( r ) $')
        line1, = fig1_ax1.plot(data['heading'], label="heading")
        fig1_ax1.legend(loc='lower right')

        fig1_ax2 = fig1.add_subplot(212)
        fig1_ax2.title.set_text('Magnetometer FFT')
        fig1_ax2.set_xlabel('Frequency (Hz)')
        fig1_ax2.set_ylabel(r'Amplitude ')
        line1, = fig1_ax2.plot(freqs, 20*scipy.log10(fft), 'x',)

        plt.show()

    def do_log_heading(self, args):
        """ Gets the heading"""
        with open ('heading_data.csv','a') as csvfile:
            writer = csv.writer(csvfile)
            while True:
                heading = self.serialComm.getHeading()
                writer.writerow([heading])

    def do_get_heading(self, args):
        """ Gets the heading"""
        heading = self.serialComm.getHeading()
        if heading:
            print "The sensed heading is : " + str(heading)

    def do_get_motor_data(self, args):
        """ Gets motor readings"""
        motor_data = self.serialComm.getMotorData()
        if motor_data:
            print "#################################################################"
            print "# WheelBot Motor Data "
            print "#################################################################"
            print "Left motor count : " + str(motor_data["LeftMotorCount"])
            print "Left motor RPM : " + str(motor_data["LeftMotorRpm"])
            print "Right motor count : " + str(motor_data["RightMotorCount"])
            print "Right motor RPM : " + str(motor_data["RightMotorRpm"])

    def do_get_active_waypoint(self, args):
        """ Gets the name of the active way point"""
        active_way_point_name = self.serialComm.getActiveWayPointName()
        if active_way_point_name:
            print "The active waypoint is : " + active_way_point_name

    def do_send_waypoint(self, args):
        """Prompts the user to enter a way-point and then sends it to the Arduino"""
        self.send_waypoint()
        return

    def do_send_csv_waypoints(self, args):
        """Opens and reads a set of way-points from a csv file and sends them to the Arduino"""
        self.send_csv_waypoints(args)
        return

    def do_test_drive(self, args):
        """Sends a command to initiate a test drive"""
        self.serialComm.commandTestDrive()

    def do_stop_test_drive(self, args):
        """Sends a command to stop the test drive"""
        self.serialComm.stopTestDrive()

    def do_manual_drive(self, args):
        """Sends a command to start manual drive mode"""
        self.serialComm.commandManualDrive()

    def do_stop_manual_drive(self, args):
        """Sends a command to stop manual drive mode"""
        self.serialComm.stopManualDrive()

    def do_command_motors(self, args):
        """Commands motors to an rpm"""
        try:
            print "Desired Left Motor RPM : "
            left_motor_rpm = float(raw_input())
            print "Desired Right Motor RPM : "
            right_motor_rpm = float(raw_input())
        except ValueError:
            print "Invalid motor commands. RPM commands must be floats."
        self.serialComm.commandMotorRpms(left_motor_rpm, right_motor_rpm)

    def do_command_motor_trajectory(self, args):
        """Commands the motors trajectory"""
        with open(args) as trajectory_file:
            trajectory_reader = csv.DictReader(trajectory_file)
            for row in trajectory_reader:
                left_motor_rpm = float(row['Left Motor Rpm'])
                right_motor_rpm = float(row['Right Motor Rpm'])
                self.serialComm.commandMotorRpms(left_motor_rpm, right_motor_rpm)
                sleep(1)

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

    def send_csv_waypoints(self, csv_file):
        with open(csv_file) as waypoints:
            waypoint_reader = csv.DictReader(waypoints)
            for row in waypoint_reader:
                way_point_name = row['Name']
                way_point_heading = float(row['Heading'])
                way_point_distance = float(row['Distance'])
                self.serialComm.sendWayPoint(way_point_name, way_point_heading, way_point_distance)

if __name__ == '__main__':
    logging.basicConfig(stream=sys.stderr, level=logging.ERROR, format='%(levelname)s:%(message)s')
    prompt = RobotTerminal()
    prompt.prompt = '>> '
    prompt.cmdloop('Welcome to the Robot Terminal. Type "help" to see a list of commands.')
