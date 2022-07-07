#! /usr/bin/env python

import rospy
import time
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from kinova_msgs.msg import PoseVelocityWithFingerVelocity
from kinova_msgs.srv import HomeArm


class ArmControlNode():
    def __init__(self):
        rospy.init_node('arm_control', anonymous=False)
        self.rate = rospy.Rate(100)
        rospy.on_shutdown(self.on_shutdown)
        self.cmd = PoseVelocityWithFingerVelocity()
        self.speed_ratio = 1.0
        self.cmd_pub = rospy.Publisher("/j2n6s300_driver/in/cartesian_velocity_with_finger_velocity", PoseVelocityWithFingerVelocity, queue_size=10)
        self.joy_sub = rospy.Subscriber('/joy_arm', Joy, self.joy_callback, queue_size=1)
        self.home_robot = rospy.ServiceProxy('/j2n6s300_driver/in/home_arm', HomeArm)
        self.init_cmd(False)
        self.keep_publishing = True
        self.send_cmd()

    def init_cmd(self, deadman_called):
        self.cmd.twist_linear_x = 0.0
        self.cmd.twist_linear_y = 0.0
        self.cmd.twist_linear_z = 0.0
        self.cmd.twist_angular_x = 0.0
        self.cmd.twist_angular_y = 0.0
        self.cmd.twist_angular_z = 0.0
        if deadman_called:
            self.cmd.finger1 = self.cmd.finger1
            self.cmd.finger2 = self.cmd.finger2
            self.cmd.finger3 = self.cmd.finger3
        else:
            self.cmd.finger1 = -100.0
            self.cmd.finger2 = -100.0
            self.cmd.finger3 = -100.0

    def joy_callback(self, msg):
        '''
        Deal with the joystick shenanigans to come up with the desired velocity in x,y,z and angular

        axes[0] -> ArrowPad (Right=-1/Left=1)
        axes[1] -> ArrowPad (Down=-1/Up=1)
        axes[2] -> LT (default=1, when pressed up to -1)
        axes[3] -> Right joystick (left=1/right=-1)
        axes[4] -> Right joystick (up=1/down=-1)
        axes[5] -> RT (default=1, when pressed up to -1)
        axes[6] -> Left joystick (left=1/right=-1)
        axes[7] -> Left joystick (up=1/down=-1)
        buttons[0] -> A
        buttons[1] -> B
        buttons[2] -> X
        buttons[3] -> Y
        buttons[4] -> LB
        buttons[5] -> RB
        buttons[6] -> Back
        buttons[7] -> Start
        buttons[8] -> Power
        buttons[9] -> Left joystick press
        buttons[10] -> Right joystick press
        '''
        # TODO do something :)
        if msg.buttons[4]:
            self.cmd.twist_linear_x = msg.axes[7]*self.speed_ratio
            self.cmd.twist_linear_y = msg.axes[6]*self.speed_ratio
            self.cmd.twist_linear_z = msg.axes[1]*self.speed_ratio
            self.cmd.twist_angular_x = msg.axes[4]*self.speed_ratio
            self.cmd.twist_angular_y = msg.axes[3]*self.speed_ratio
            self.cmd.twist_angular_z = msg.axes[0]*self.speed_ratio
            if msg.buttons[0]:
                # self.cmd.fingers_closure_percentage = 100
                self.cmd.finger1 = 100.0
                self.cmd.finger2 = 100.0
                self.cmd.finger3 = 100.0
            elif msg.buttons[2]:
                self.cmd.finger1 = -100.0
                self.cmd.finger2 = -100.0
                self.cmd.finger3 = -100.0
            elif msg.buttons[1]:
                self.cmd.finger1 = 100.0
                self.cmd.finger2 = 100.0
                self.cmd.finger3 = -100.0
            elif msg.buttons[3]:
                self.cmd.finger1 = -100.0
                self.cmd.finger2 = -100.0
                self.cmd.finger3 = -100.0
            if msg.buttons[7]:
                _ = self.home_robot()
        else:
            self.init_cmd(True)

    def send_cmd(self):
        while self.keep_publishing:
            self.cmd_pub.publish(self.cmd)
            self.rate.sleep()

    def on_shutdown(self):
        self.init_cmd(False)
        self.keep_publishing = False


if __name__ == '__main__':
    try:
        node = ArmControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
