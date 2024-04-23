#!/usr/bin/env python3

import rospy
from threading import Thread
from onrobot_rg_control.msg import OnRobotRGOutputStamped, OnRobotRGInputStamped

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
        
        self._init_last_command()
        self.pub = rospy.Publisher(
            'OnRobotRGOutputStamped', OnRobotRGOutputStamped, queue_size=1)
        self.pub_thread = Thread(target=self.publisher, args=())
        self.pub_thread.start()
        self.listener()
        
    def _init_last_command(self):
        init_state = rospy.wait_for_message('OnRobotRGInputStamped', OnRobotRGInputStamped)
        self.last_command = OnRobotRGOutputStamped()
        self.last_command.rGFR = 400
        self.last_command.rGWD = init_state.gWDF
        self.last_command.rCTR = 16

    def genCommand(self, cmd):
        if cmd == 'c':
            # self.last_command.rGFR = 400
            self.last_command.rGWD = 0
            self.last_command.rCTR = 16
        elif cmd == 'o':
            # self.last_command.rGFR = 400
            self.last_command.rGWD = self.max_width
            self.last_command.rCTR = 16
        elif cmd == 'i':
            self.last_command.rGFR += 25
            self.last_command.rGFR = min(self.max_force, self.last_command.rGFR)
            self.last_command.rCTR = 16
        elif cmd == 'd':
            self.last_command.rGFR -= 25
            self.last_command.rGFR = max(0, self.last_command.rGFR)
            # self.last_command.rCTR = 16
        elif cmd.startswith('f'):
            try:
                force_cmd = int(cmd[1:])
                print(cmd, force_cmd)
                self.last_command.rGFR = max(0, min(force_cmd, self.max_force))
            except ValueError:
                pass
        else:
            # If the self.last_command entered is a int, assign this value to rGWD
            try:
                # self.last_command.rGFR = 400
                self.last_command.rGWD = min(self.max_width, int(cmd))
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
        strAskForCommand += '(0 - max width): Set position to value\n'
        strAskForCommand += 'f(0-max force): Set force to value\n'
        strAskForCommand += 'i: Increase force by 25\n'
        strAskForCommand += 'd: Decrease force by 25\n'

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
