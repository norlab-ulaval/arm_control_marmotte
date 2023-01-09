#! /usr/bin/env python

import rospy
import time
import json
import numpy as np
from math import sin, cos, degrees, radians
import actionlib
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Quaternion
from kortex_driver.msg import *
from kortex_driver.srv import *
import rospkg
from tf.transformations import euler_from_quaternion, quaternion_from_euler


'''
    Logitech Joystick Button Mapping

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

root = rospkg.RosPack().get_path('arm_control_marmotte')
sequence_files = {
    'grab_dumbbell': root+"/sequences/grab_dumbbell.json", 
    'analyze_dumbbell': root+"/sequences/analyze_dumbbell.json", 
    'open_door': root+"/sequences/open_door.json"  
    }
viewpoint_joint_state = [195.89, 168.79, 47.21, 185.70, 65.55, 139.11]
dumbell_joint_state = [189.1484, 261.7324,  58.8002, 155.5835,  33.3919, 103.7543]

class ArmControlNode():

    def __init__(self):
        rospy.init_node('arm_control', anonymous=False)
        rate = rospy.Rate(100)

        np.set_printoptions(precision=4, suppress=True)

        # Variables
        self.HOME_ACTION_IDENTIFIER = 2
        self.joint_angles = []
        self.cartesian_pose = []
        self.finger_positions = []
        self.cmd = TwistCommand()
        self.init_cmd()

        # Arguments
        self.speed_ratio = rospy.get_param("speed_ratio", default=1.0)
        self.prefix = rospy.get_param("arm_prefix", default="my_gen3")
        
        # Publishers / Subscribers
        self.cmd_pub = rospy.Publisher("/" + self.prefix + "/in/cartesian_velocity", TwistCommand, queue_size=10)
        self.joy_sub = rospy.Subscriber('/joy_arm', Joy, self.joy_callback, queue_size=1)

        self.action_topic_sub = rospy.Subscriber("/" + self.prefix + "/action_topic", ActionNotification,
                                                 self.cb_action_topic)
        self.last_action_notif_type = None
        # self.joint_angles_sub = rospy.Subscriber('/' + self.prefix +'driver/out/joint_angles', JointAngles, callback=self.joint_angles_callback, queue_size=1)
        # self.cartesian_pose_sub = rospy.Subscriber('/' + self.prefix +'driver/out/tool_pose', PoseStamped, callback=self.cartesian_pose_callback, queue_size=1)
        # self.finger_pose_sub = rospy.Subscriber('/' + self.prefix +'driver/out/finger_position', FingerPosition, callback=self.finger_positions_callback, queue_size=1)

        # Service calls
        clear_faults_full_name = '/' + self.prefix + '/base/clear_faults'
        rospy.wait_for_service(clear_faults_full_name)
        self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

        read_action_full_name = '/' + self.prefix + '/base/read_action'
        rospy.wait_for_service(read_action_full_name)
        self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

        execute_action_full_name = '/' + self.prefix + '/base/execute_action'
        rospy.wait_for_service(execute_action_full_name)
        self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

        set_cartesian_reference_frame_full_name = '/' + self.prefix + '/control_config/set_cartesian_reference_frame'
        rospy.wait_for_service(set_cartesian_reference_frame_full_name)
        self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name,
                                                                SetCartesianReferenceFrame)

        send_gripper_command_full_name = '/' + self.prefix + '/base/send_gripper_command'
        rospy.wait_for_service(send_gripper_command_full_name)
        self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

        activate_publishing_of_action_notification_full_name = '/' + self.prefix + '/base/activate_publishing_of_action_topic'
        rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
        self.activate_publishing_of_action_notification = rospy.ServiceProxy(
            activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)


        # self.home_robot = rospy.ServiceProxy('/' + self.prefix + 'driver/in/home_arm', HomeArm)
        # self.clear_trajectories = rospy.ServiceProxy('/' + self.prefix + 'driver/in/clear_trajectories', ClearTrajectories)
        # self.zero_torques = rospy.ServiceProxy('/' + self.prefix + 'driver/in/set_zero_torques', ZeroTorques)
        # self.start_force_control = rospy.ServiceProxy('/' + self.prefix + 'driver/in/start_force_control', Start)
        # self.stop_force_control = rospy.ServiceProxy('/' + self.prefix + 'driver/in/stop_force_control', Stop)
        # self.start_robot = rospy.ServiceProxy('/' + self.prefix + 'driver/in/start', Start)
        # self.stop_robot = rospy.ServiceProxy('/' + self.prefix + 'driver/in/stop', Stop)

        # # Action clients
        # self.pose_action_client = actionlib.SimpleActionClient('/' + self.prefix + 'driver/pose_action/tool_pose', ArmPoseAction)
        # self.pose_action_client.wait_for_server()
        # self.joints_action_client = actionlib.SimpleActionClient('/' + self.prefix + 'driver/joints_action/joint_angles', ArmJointAnglesAction)
        # self.joints_action_client.wait_for_server()
        # self.gripper_client = actionlib.SimpleActionClient('/' + self.prefix + 'driver/fingers_action/finger_positions', SetFingersPositionAction)
        # self.gripper_client.wait_for_server()

        # self.start_robot()
        while not rospy.is_shutdown():
            self.cmd_pub.publish(self.cmd)
            rate.sleep()

        self.init_cmd()

    def init_cmd(self):
        self.cmd.twist.linear_x = 0.0
        self.cmd.twist.linear_y = 0.0
        self.cmd.twist.linear_z = 0.0
        self.cmd.twist.angular_x = 0.0
        self.cmd.twist.angular_y = 0.0
        self.cmd.twist.angular_z = 0.0
        self.cmd.duration = 0

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                return False
            else:
                time.sleep(0.01)
    # def move_joints(self, angle_set):
    #
    #     goal = ArmJointAnglesGoal()
    #
    #     goal.angles.joint1 = angle_set[0]
    #     goal.angles.joint2 = angle_set[1]
    #     goal.angles.joint3 = angle_set[2]
    #     goal.angles.joint4 = angle_set[3]
    #     goal.angles.joint5 = angle_set[4]
    #     goal.angles.joint6 = angle_set[5]
    #
    #     self.joints_action_client.send_goal(goal)
    #     if self.joints_action_client.wait_for_result(rospy.Duration(200.0)):
    #         return self.joints_action_client.get_result()
    #     else:
    #         self.joints_action_client.cancel_all_goals()
    #         rospy.logwarn('        the joint angle action timed-out')
    #         return None

    
    # def move_cartesian(self, pose):
    #
    #     goal = ArmPoseGoal()
    #     goal.pose.header = Header(frame_id=(self.prefix + "link_base"))
    #     goal.pose.pose.position = Point(x=pose[0], y=pose[1], z=pose[2])
    #     orientation = quaternion_from_euler(radians(pose[3]), radians(pose[4]), radians(pose[5]))
    #     goal.pose.pose.orientation = Quaternion(x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])
    #
    #     self.pose_action_client.send_goal(goal)
    #
    #     if self.pose_action_client.wait_for_result(rospy.Duration(200.0)):
    #         return self.pose_action_client.get_result()
    #     else:
    #         self.pose_action_client.cancel_all_goals()
    #         rospy.logwarn('        the cartesian action timed-out')
    #         return None

        
    # def move_gripper(self, finger_positions):
    #
    #     goal = SetFingersPositionGoal()
    #     goal.fingers.finger1 = float(finger_positions[0])
    #     goal.fingers.finger2 = float(finger_positions[1])
    #     goal.fingers.finger3 = float(finger_positions[2])
    #
    #     self.gripper_client.send_goal(goal)
    #
    #     if self.gripper_client.wait_for_result(rospy.Duration(200.0)):
    #         return self.gripper_client.get_result()
    #     else:
    #         self.gripper_client.cancel_all_goals()
    #         rospy.logwarn('        the gripper action timed-out')
    #         return None


    # def execute_sequence(self, sequence_name):
    #     file = sequence_files[sequence_name]
    #     with open(file) as fp:
    #         sequence = json.load(fp)
    #
    #     for i, position in enumerate(sequence["positions"]):
    #
    #         if position["type"] == "joints":
    #             if position["reference"] == "relative":
    #                 self.move_joints(self.joint_angles + position["value"])
    #             elif position["reference"] == "absolute":
    #                 self.move_joints(position["value"])
    #
    #         elif position["type"] == "cartesian":
    #             if position["reference"] == "relative":
    #                 self.move_cartesian(self.cartesian_pose + position["value"])
    #             elif position["reference"] == "absolute":
    #                 self.move_cartesian(position["value"])
    #
    #         elif position["type"] == "gripper":
    #             if position["reference"] == "relative":
    #                 print("Relative position not supported in finger mode.")
    #             elif position["reference"] == "absolute":
    #                 self.move_gripper(position["value"])
    #
    #         print("Position {} achieved.".format(i))
    #         time.sleep(0.5)


    def joy_callback(self, msg):

        if msg.buttons[4]:
            # Arm control joysticks
            self.cmd.twist.linear_x = msg.axes[7]*self.speed_ratio
            self.cmd.twist.linear_y = msg.axes[6]*self.speed_ratio
            self.cmd.twist.linear_z = msg.axes[1]*self.speed_ratio
            self.cmd.twist.angular_x = msg.axes[4]*self.speed_ratio
            self.cmd.twist.angular_y = msg.axes[3]*self.speed_ratio
            self.cmd.twist.angular_z = -msg.axes[0]*self.speed_ratio

            # # Fingers control
            # if msg.axes[5] < 0:
            #     # self.cmd.fingers_closure_percentage = 100
            #     self.cmd.finger1 = 100.0
            #     self.cmd.finger2 = 100.0
            #     self.cmd.finger3 = 100.0
            # elif msg.axes[2] < 0:
            #     self.cmd.finger1 = -100.0
            #     self.cmd.finger2 = -100.0
            #     self.cmd.finger3 = -100.0
            # else:
            #     self.cmd.finger1 = 0.0
            #     self.cmd.finger2 = 0.0
            #     self.cmd.finger3 = 0.0

            # Predefined positions
            if msg.buttons[7]:
                self.last_action_notif_type=None
                req = ReadActionRequest()
                req.input.identifier = self.HOME_ACTION_IDENTIFIER
                rospy.loginfo(req)
                try:
                    res = self.read_action(req)
                except rospy.ServiceException:
                    rospy.logerr("Failed to call ReadAction")
                    return False
                # Execute the HOME action if we could read it
                else:
                    # What we just read is the input of the ExecuteAction service
                    req = ExecuteActionRequest()
                    req.input = res.output
                    rospy.loginfo("Sending the robot home...")
                    try:
                        self.execute_action(req)
                    except rospy.ServiceException:
                        rospy.logerr("Failed to call ExecuteAction")
                        return False
                    else:
                        return self.wait_for_action_end_or_abort()


            # elif msg.buttons[6]:
            #     _ = self.start_force_control()
            # elif msg.buttons[3]:    # Y
            #     _ = self.move_joints(viewpoint_joint_state)
            # elif msg.buttons[0]:    # A
            #     _ = self.execute_sequence("analyze_dumbbell")
            # elif msg.buttons[1]:    # B
            #     _ = self.execute_sequence("open_door")
            # elif msg.buttons[2]:    # X
            #     _ = self.execute_sequence("grab_dumbbell")

            # if msg.buttons[8]:     # power
            #     _ = self.stop_robot()
            #     _ = self.clear_trajectories()
        else:
            # _ = self.start_robot()
            self.init_cmd()

        # if msg.buttons[5]:
        #     print(" Joint positions : {} \n  Cartesian pose : {}\nFinger positions : {}\n".format(self.joint_angles, self.cartesian_pose, self.finger_positions))

    # def joint_angles_callback(self, msg):
    #     self.joint_angles = np.array([msg.joint1, msg.joint2, msg.joint3, msg.joint4, msg.joint5, msg.joint6])

    # def cartesian_pose_callback(self, msg):
    #     euler_angles = euler_from_quaternion((msg.pose.orientation.x, msg.pose.orientation.y,
    #                                           msg.pose.orientation.z, msg.pose.orientation.w))
    #     self.cartesian_pose = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
    #                                     degrees(euler_angles[0]), degrees(euler_angles[1]), degrees(euler_angles[2])])

    # def finger_positions_callback(self, msg):
    #     self.finger_positions = np.array([msg.finger1, msg.finger2, msg.finger3])


if __name__ == '__main__':
    try:
        node = ArmControlNode()
    except rospy.ROSInterruptException:
        pass
