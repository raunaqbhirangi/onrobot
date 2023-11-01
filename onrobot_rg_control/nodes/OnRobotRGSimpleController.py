#!/usr/bin/env python3

import rospy
from threading import Thread
from onrobot_rg_control.msg import OnRobotRGOutputStamped

class OnrobotController:
    def __init__(self):
        self.gtype = rospy.get_param("/onrobot/gripper", "rg2")
        if self.gtype == 'rg2':
            self.max_force = 400
            self.max_width = 1100
        elif self.gtype == 'rg6':
            self.max_force = 1200
            self.max_width = 1600
        else:
            rospy.signal_shutdown(
                rospy.get_name() +
                ": Select the gripper type from rg2 or rg6.")
        rospy.init_node(
            'OnRobotRGSimpleController',
            anonymous=True,
            log_level=rospy.DEBUG)
        self.pub = rospy.Publisher(
            'OnRobotRGOutputStamped', OnRobotRGOutputStamped, queue_size=1)
        self.last_command = OnRobotRGOutputStamped()
        self.genCommand("o")
        self.pub_thread = Thread(target=self.publisher, args=())
        self.pub_thread.start()
        self.listener()
        

    def genCommand(self, char):
        if char == 'c':
            self.last_command.rGFR = 400
            self.last_command.rGWD = 0
            self.last_command.rCTR = 16
        elif char == 'o':
            self.last_command.rGFR = 400
            self.last_command.rGWD = self.max_width
            self.last_command.rCTR = 16
        elif char == 'i':
            self.last_command.rGFR += 25
            self.last_command.rGFR = min(self.max_force, self.last_command.rGFR)
            self.last_command.rCTR = 16
        elif char == 'd':
            self.last_command.rGFR -= 25
            self.last_command.rGFR = max(0, self.last_command.rGFR)
            self.last_command.rCTR = 16
        else:
            # If the self.last_command entered is a int, assign this value to rGWD
            try:
                self.last_command.rGFR = 400
                self.last_command.rGWD = min(self.max_width, int(char))
                self.last_command.rCTR = 16
            except ValueError:
                pass
        # self.last_command.header.stamp = rospy.get_rostime()

    def askForCommand(self):
        """ Asks the user for a command to send to the gripper.

            Args:
                command (OnRobotRGOutputStamped): command to be sent

            Returns:
                input(strAskForCommand) (str): input command strings
        """

        currentCommand = 'Simple OnRobot RG Controller\n-----\nCurrent command:'
        currentCommand += ' rGFR = ' + str(self.last_command.rGFR)
        currentCommand += ', rGWD = ' + str(self.last_command.rGWD)
        currentCommand += ', rCTR = ' + str(self.last_command.rCTR)

        rospy.loginfo(currentCommand)

        strAskForCommand = '-----\nAvailable commands\n\n'
        strAskForCommand += 'c: Close\n'
        strAskForCommand += 'o: Open\n'
        strAskForCommand += '(0 - max width): Go to that position\n'
        strAskForCommand += 'i: Increase force\n'
        strAskForCommand += 'd: Decrease force\n'

        strAskForCommand += '-->'

        return input(strAskForCommand)

    def listener(self):
        while not rospy.is_shutdown():
            self.genCommand(self.askForCommand())

    def publisher(self):
        while not rospy.is_shutdown():
            self.last_command.header.stamp = rospy.get_rostime()
            self.pub.publish(self.last_command)
            rospy.sleep(0.1)


if __name__ == '__main__':
    OnrobotController()
