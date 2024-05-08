import signal
import numpy as np
import rospy
from onrobot_rg_control.msg import OnRobotRGInputStamped
from onrobot_rg_control.msg import OnRobotRGOutputStamped
from threading import Thread, Event

'''
    Input message:
        gFOF: Current fingertip offset in 1/10 millimeters. The value is a signed two's complement number.
        gGWD : Current width between the gripper fingers in 1/10 millimeters.
        gSTA: Current device status, indicates the status of the gripper and its motion.
            Bit       - Name              - Description
            # 0 (LSB)   - Busy              - High (1) when a motion is ongoing, low (0) when not. The gripper will only accept new commands when this flag is low.
            # 1         - Grip detected     - High (1) when an internal- or external grip is detected.
            # 2         - S1 pushed         - High (1) when safety switch 1 is pushed.
            # 3         - S1 trigged        - High (1) when safety circuit 1 is activated. The gripper will not move while this flag is high.
            # 4         - S2 pushed         - High (1) when safety switch 2 is pushed.
            # 5         - S2 trigged        - High (1) when safety circuit 2 is activated. The gripper will not move while this flag is high.
            # 6         - Safety error      - High (1) when on power on any of the safety switch is pushed.
            # 7 - 15    - Reserved          - Not used.
        gWDF : Current width between the gripper fingers in 1/10 millimeters, considering set offset

    Output message:
        rGFR: The target force to be reached when gripping and holding a workpiece, in 1/10th Newtons. (0-400)
        rGWD: The target width between the finger to be moved to and maintained, in 1/10th millimeters, corrected for fingertip offset
        rCTR: Control field used to start and stop gripper motion.
            0x0001 - grip - Start the motion, with the preset target force and width (w/o fingertip offset). Ignored if status busy.
            0x0008 - stop
            0x0010 - grip_w_offset
'''

ONROBOT_INPUT_TOPIC = '/OnRobotRGInputStamped' 
ONROBOT_OUTPUT_TOPIC = '/OnRobotRGOutputStamped' 

# Maximum permitted values
MAX_WIDTH = 1100
MAX_TORQUE = 400

DEFAULT_VAL = None

class OnrobotController:
    def __init__(self, record_type):
        try:
            rospy.init_node("onrobot_gripper_node")
        except:
            pass
        
        # TODO: Fix these topics
        rospy.Subscriber(ONROBOT_INPUT_TOPIC, OnRobotRGInputStamped, self._sub_callback_gripper_state)

        self.gripper_comm_publisher = rospy.Publisher(ONROBOT_OUTPUT_TOPIC, OnRobotRGOutputStamped, queue_size=-1)
        init_state = rospy.wait_for_message('/OnRobotRGInputStamped', OnRobotRGInputStamped)
        self.command = OnRobotRGOutputStamped()
        # TODO: Add option for setting different force values
        self.command.rGFR = 400
        self.command.rGWD = init_state.gWDF
        self.command.rCTR = 16
        if record_type is None:
            self.pub_thread = Thread(target=self.publisher, args=())
            self.pub_thread.start()

        
        self.current_gripper_state = init_state
        # self.grav_comp = DEFAULT_VAL
        # self.cmd_joint_state = DEFAULT_VAL

    def _sub_callback_gripper_state(self, data):
        self.current_gripper_state = data

    def _sub_callback_cmd_joint_state(self, data):
        self.cmd_joint_state = data
    
    def gripper_width(self, desired_action=0, absolute=True):
        if self.current_gripper_state == DEFAULT_VAL:
            print('No gripper data received!')
            return
        current_state = self.current_gripper_state

        if absolute is True:
            desired_width = desired_action
        else:
            desired_width = desired_action + current_state.gWDF

        action = self._clip(desired_action, MAX_WIDTH)

        self.command.rGWD = int(action)
    
    def _clip(self, action, value):
        return np.clip(action, -value, value)

    def log_current_pose(self, log_file):
        if self.current_gripper_state is not DEFAULT_VAL:
            current_angles = self.current_gripper_state.position
            current_velocity = self.current_gripper_state.velocity
            current_torque = self.current_gripper_state.effort
        else:
            current_angles = DEFAULT_VAL
            current_velocity = DEFAULT_VAL
            current_torque = DEFAULT_VAL

        if self.grav_comp is not DEFAULT_VAL:
            grav_comp_torques = self.grav_comp.effort
        else: 
            grav_comp_torques = DEFAULT_VAL

        if self.cmd_joint_state is not DEFAULT_VAL:
            cmd_joint_position = self.cmd_joint_state.position
            cmd_joint_torque = self.cmd_joint_state.effort
        else:
            cmd_joint_position = DEFAULT_VAL
            cmd_joint_torque = DEFAULT_VAL

        time = get_datetime()

        print('Write done at:', time)

        with open(log_file, 'a') as csvfile:
            log_writer = csv.writer(csvfile, delimiter=' ')

            log_writer.writerow(
                [time]
                + [current_angles]
                + [current_velocity] 
                + [current_torque]
                + [grav_comp_torques]
                + [cmd_joint_position]
                + [cmd_joint_torque]
                )

    def publisher(self):
        while not rospy.is_shutdown():
            self.command.header.stamp = rospy.get_rostime()
            self.gripper_comm_publisher.publish(self.command)
            rospy.sleep(0.1)