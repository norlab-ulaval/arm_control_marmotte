#! /usr/bin/env python

import rospy
import time
import json
import numpy as np
import math
import actionlib
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Quaternion
from kinova_msgs.msg import PoseVelocityWithFingerVelocity, ArmPoseAction, ArmPoseGoal, ArmJointAnglesAction, ArmJointAnglesGoal
from kinova_msgs.srv import HomeArm


prefix = "j2n6s300_"
sequence_files = {'dumbell': "./sequences/dumbell.json"}
viewpoint_joint_state = [3.419, 2.946, 0.824, 3.241, 1.144, 2.428]
dumbell_joint_state = [3.506, 4.538, 0.929, 2.919, 0.644, 3.337]

class ArmControlNode():
    def __init__(self):
        rospy.init_node('arm_control', anonymous=False)
        self.rate = rospy.Rate(100)
        rospy.on_shutdown(self.on_shutdown)
        self.cmd = PoseVelocityWithFingerVelocity()
        self.speed_ratio = 1.0
        self.cmd_pub = rospy.Publisher("/" + prefix + "driver/in/cartesian_velocity_with_finger_velocity", PoseVelocityWithFingerVelocity, queue_size=10)
        self.joy_sub = rospy.Subscriber('/joy_arm', Joy, self.joy_callback, queue_size=1)
        self.joint_state_sub = rospy.Subscriber('/' + prefix +'driver/out/joint_state', JointState, callback=self.joint_state_callback, queue_size=1)
        self.joint_state = JointState()

        self.home_robot = rospy.ServiceProxy('/' + prefix + 'driver/in/home_arm', HomeArm)
        #self.pose_action_client = actionlib.SimpleActionClient('/' + prefix + 'driver/pose_action/tool_pose', Arm_KinovaPoseAction)
        #self.pose_action_client = actionlib.SimpleActionClient('/' + prefix + 'driver/pose_action/tool_pose', ArmPoseAction)
        #self.pose_action_client.wait_for_server()
        self.joints_action_client = actionlib.SimpleActionClient('/' + prefix + 'driver/joints_action/joint_angles', ArmJointAnglesAction)
        self.joints_action_client.wait_for_server()
        self.init_cmd()
        self.keep_publishing = True
        self.send_cmd()


    def init_cmd(self):
        self.cmd.twist_linear_x = 0.0
        self.cmd.twist_linear_y = 0.0
        self.cmd.twist_linear_z = 0.0
        self.cmd.twist_angular_x = 0.0
        self.cmd.twist_angular_y = 0.0
        self.cmd.twist_angular_z = 0.0
        self.cmd.finger1 = 0.0
        self.cmd.finger2 = 0.0
        self.cmd.finger3 = 0.0


    def cartesian_pose_client(self, position, orientation):

        goal = ArmPoseGoal()
        goal.pose.header = Header(frame_id=(prefix + "link_base"))
        goal.pose.pose.position = Point(x=position[0], y=position[1], z=position[2])
        goal.pose.pose.orientation = Quaternion(x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

        print('goal.pose in client 1: {}'.format(goal.pose.pose)) # debug

        self.pose_action_client.send_goal(goal)

        if self.pose_action_client.wait_for_result(rospy.Duration(200.0)):
            return self.pose_action_client.get_result()
        else:
            self.pose_action_client.cancel_all_goals()
            print('        the cartesian action timed-out')
            return None

    
    def move_joints(self, angle_set):

        goal = ArmJointAnglesGoal()

        goal.angles.joint1 = math.degrees(angle_set[0])
        goal.angles.joint2 = math.degrees(angle_set[1])
        goal.angles.joint3 = math.degrees(angle_set[2])
        goal.angles.joint4 = math.degrees(angle_set[3])
        goal.angles.joint5 = math.degrees(angle_set[4])
        goal.angles.joint6 = math.degrees(angle_set[5])
        #goal.angles.joint7 = angle_set[6]

        self.joints_action_client.send_goal(goal)
        if self.joints_action_client.wait_for_result(rospy.Duration(50.0)):
            return self.joints_action_client.get_result()
        else:
            print('        the joint angle action timed-out')
            self.joints_action_client.cancel_all_goals()
            return None


    def execute_sequence(self, sequence_name):
        file = sequence_files[sequence_name]
        with open(file) as fp:
            sequence = json.load(fp)

        for i, angle_set in enumerate(sequence):
            self.move_joints(angle_set)
            print("Position ", i, " achieved.")

    
    def EulerXYZ2Quaternion(EulerXYZ_):
        tx_, ty_, tz_ = EulerXYZ_[0:3]
        sx = math.sin(0.5 * tx_)
        cx = math.cos(0.5 * tx_)
        sy = math.sin(0.5 * ty_)
        cy = math.cos(0.5 * ty_)
        sz = math.sin(0.5 * tz_)
        cz = math.cos(0.5 * tz_)

        qx_ = sx * cy * cz + cx * sy * sz
        qy_ = -sx * cy * sz + cx * sy * cz
        qz_ = sx * sy * cz + cx * cy * sz
        qw_ = -sx * sy * sz + cx * cy * cz

        Q_ = [qx_, qy_, qz_, qw_]
        return Q_


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
        if msg.buttons[4]:

            # Arm control joysticks
            self.cmd.twist_linear_x = msg.axes[7]*self.speed_ratio
            self.cmd.twist_linear_y = msg.axes[6]*self.speed_ratio
            self.cmd.twist_linear_z = msg.axes[1]*self.speed_ratio
            self.cmd.twist_angular_x = msg.axes[4]*self.speed_ratio
            self.cmd.twist_angular_y = msg.axes[3]*self.speed_ratio
            self.cmd.twist_angular_z = msg.axes[0]*self.speed_ratio

            # Fingers control
            if msg.axes[5] < 0:
                # self.cmd.fingers_closure_percentage = 100
                self.cmd.finger1 = 100.0
                self.cmd.finger2 = 100.0
                self.cmd.finger3 = 100.0
            elif msg.axes[2] < 0:
                self.cmd.finger1 = -100.0
                self.cmd.finger2 = -100.0
                self.cmd.finger3 = -100.0
            else:
                self.cmd.finger1 = 0.0
                self.cmd.finger2 = 0.0
                self.cmd.finger3 = 0.0

            # Predefined positions
            if msg.buttons[7]:
                _ = self.home_robot()
            elif msg.buttons[3]:
                _ = self.move_joints(viewpoint_joint_state)
            elif msg.buttons[0]:
                _ = self.move_joints(dumbell_joint_state)
                # self.show_dumbell_seq()

        else:
            self.init_cmd()

        if msg.buttons[5]:
            print(self.joint_state.position)

    def show_dumbell_seq(self):
        _ = self.move_joints(dumbell_joint_state)
        self.cmd.twist_angular_z = -1.0
        time.sleep(2)
        self.cmd.twist_angular_z = 0.0
        self.cmd.finger1 = -100.0
        self.cmd.finger2 = -100.0
        self.cmd.finger3 = -100.0
        time.sleep(2)
        self.cmd.finger1 = 0.0
        self.cmd.finger2 = 0.0
        self.cmd.finger3 = 0.0
        _ = self.move_joints(viewpoint_joint_state)
    
    def joint_state_callback(self, msg):
        self.joint_state = msg

    def send_cmd(self):
        while self.keep_publishing:
            self.cmd_pub.publish(self.cmd)
            self.rate.sleep()

    def on_shutdown(self):
        self.init_cmd()
        self.keep_publishing = False


if __name__ == '__main__':
    try:
        node = ArmControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
