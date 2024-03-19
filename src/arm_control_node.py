#! /usr/bin/env python

import rospy
import time
import json
import numpy as np
from math import sin, cos, degrees, radians
import actionlib
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Header, UInt8
from geometry_msgs.msg import Point, Quaternion
from kortex_driver.msg import *
from kortex_driver.srv import *
from arm_control_marmotte.srv import sendRobotToPosition, sendRobotToPositionResponse
import rospkg
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from dynamic_reconfigure.server import Server
from arm_control_marmotte.cfg import speed_ratiosConfig

"""
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
"""
MAX_FINGER_POS = 1.0

root = rospkg.RosPack().get_path("arm_control_marmotte")
sequence_files = {
    'grab_dumbbell': root+"/sequences/grab_dumbbell.json", 
    'analyze_dumbbell': root+"/sequences/analyze_dumbbell.json", 
    'open_door': root+"/sequences/open_door.json",
    'panoramic_image': root+"/sequences/panoramic_image_2.json",
    'test_sequence': root+"/sequences/test_sequence.json"  
    }
position_files = {
    "home": root + "/positions/home.json",
    "upstairs": root + "/positions/upstairs.json",
    "downstairs": root + "/positions/downstairs.json",
    "sleep": root + "/positions/sleep.json",
    "reach_test": root + "/positions/reach_test.json",
    "table": root + "/positions/table.json",
    "door": root + "/positions/door.json",
}
# viewpoint_joint_state = [195.89, 168.79, 47.21, 185.70, 65.55, 139.11]
# dumbell_joint_state = [189.1484, 261.7324,  58.8002, 155.5835,  33.3919, 103.7543]
home_joint_angles = [0.0, 0.0, 180.0, 215.0, 0.0, 52.5, 90.0]
retract_pose_joint_angles = [0.0, 330.0, 180.0, 215.0, 0.0, 305.0, 90.0]
test_cartesian_pose = [0.1, -0.45, 0.55, -90.0, -180.0, 180.0]
reach_test_pose = [90.0, 90.0, 180.0, 0.0, 0.0, 0.0, 90.0]
table_test_pose = [90.0, 30.0, 180.0, -35.0, 0.0, -115.0, 90.0]
stair_test_pose = [90.0, 65.0, 180.0, -30.0, 0.0, -5.0, 90.0]


class ArmControlNode:

    def __init__(self):
        rospy.init_node("arm_control", anonymous=False)
        self.rate = rospy.Rate(100)

        np.set_printoptions(precision=4, suppress=True)

        # Variables
        self.current_joint_angles = []
        self.current_cartesian_pose = []
        self.current_finger_positions = []
        self.cartesian_cmd = TwistCommand()
        self.joint_cmd = Base_JointSpeeds()
        self.init_cmd()
        self.cartesian_deadman = False
        self.joint_deadman = False
        self.fingers_moving = False

        # Arguments
        self.cartesian_speed_ratio = rospy.get_param(
            "~cartesian_speed_ratio", default=0.1
        )
        rospy.loginfo("cartesian_speed_ratio: ")
        rospy.loginfo(self.cartesian_speed_ratio)

        self.joint_speed_ratio = rospy.get_param("~joint_speed_ratio", default=0.4)
        rospy.loginfo("joint_speed_ratio: ")
        rospy.loginfo(self.joint_speed_ratio)

        self.prefix = rospy.get_param("~group_name", default="")
        rospy.loginfo("prefix used: ")
        rospy.loginfo(self.prefix)

        # Publishers / Subscribers
        self.cartesian_cmd_pub = rospy.Publisher(
            "/" + self.prefix + "/in/cartesian_velocity", TwistCommand, queue_size=1
        )
        self.joint_cmd_pub = rospy.Publisher(
            "/" + self.prefix + "/in/joint_velocity", Base_JointSpeeds, queue_size=1
        )
        self.joy_sub = rospy.Subscriber(
            "/joy_arm", Joy, self.joy_callback, queue_size=1
        )
        self.sequence_idx_pub = rospy.Publisher(
            f"/{self.prefix}/sequence_idx",
            UInt8,
            queue_size=1,
        )

        self.action_topic_sub = rospy.Subscriber(
            "/" + self.prefix + "/action_topic",
            ActionNotification,
            self.cb_action_topic,
        )
        self.last_action_notif_type = None
        self.base_feedback_sub = rospy.Subscriber(
            "/" + self.prefix + "/base_feedback",
            BaseCyclic_Feedback,
            callback=self.base_feedback_callback,
            queue_size=1,
        )
        self.joint_state_sub = rospy.Subscriber(
            "/" + self.prefix + "/base_feedback/joint_state",
            JointState,
            callback=self.joint_state_callback,
            queue_size=1,
        )
        # self.joint_state_sub = rospy.Subscriber('/' + self.prefix +'/joint_states', FingerPosition, callback=self.finger_positions_callback, queue_size=1)

        # Service calls
        clear_faults_full_name = "/" + self.prefix + "/base/clear_faults"
        rospy.wait_for_service(clear_faults_full_name)
        self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

        read_action_full_name = "/" + self.prefix + "/base/read_action"
        rospy.wait_for_service(read_action_full_name)
        self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

        execute_action_full_name = "/" + self.prefix + "/base/execute_action"
        rospy.wait_for_service(execute_action_full_name)
        self.execute_action = rospy.ServiceProxy(
            execute_action_full_name, ExecuteAction
        )

        set_cartesian_reference_frame_full_name = (
            "/" + self.prefix + "/control_config/set_cartesian_reference_frame"
        )
        rospy.wait_for_service(set_cartesian_reference_frame_full_name)
        self.set_cartesian_reference_frame = rospy.ServiceProxy(
            set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame
        )

        send_gripper_command_full_name = (
            "/" + self.prefix + "/base/send_gripper_command"
        )
        rospy.wait_for_service(send_gripper_command_full_name)
        self.send_gripper_command = rospy.ServiceProxy(
            send_gripper_command_full_name, SendGripperCommand
        )

        activate_publishing_of_action_notification_full_name = (
            "/" + self.prefix + "/base/activate_publishing_of_action_topic"
        )
        rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
        self.activate_publishing_of_action_notification = rospy.ServiceProxy(
            activate_publishing_of_action_notification_full_name,
            OnNotificationActionTopic,
        )

        validate_waypoint_list_full_name = (
            "/" + self.prefix + "/base/validate_waypoint_list"
        )
        rospy.wait_for_service(validate_waypoint_list_full_name)
        self.validate_waypoint_list = rospy.ServiceProxy(
            validate_waypoint_list_full_name, ValidateWaypointList
        )

        sendRobotToPosition_sevice = rospy.Service(
            "send_robot_to_position",
            sendRobotToPosition,
            self.send_position_service_callback,
        )

        # Activate the publishing of the ActionNotification
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
            return False
        else:
            rospy.loginfo("Successfully activated the Action Notifications!")

        srv = Server(speed_ratiosConfig, self.dyn_reconfigure_callback)

        while not rospy.is_shutdown():
            if self.cartesian_deadman:
                req = SetCartesianReferenceFrameRequest()
                req.input.reference_frame = (
                    CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_TOOL
                )
                self.set_cartesian_reference_frame(req)
                self.cartesian_cmd_pub.publish(self.cartesian_cmd)
            elif self.joint_deadman:
                self.joint_cmd_pub.publish(self.joint_cmd)
            self.rate.sleep()

        self.init_cmd()

    def init_cmd(self):
        self.cartesian_cmd.twist.linear_x = 0.0
        self.cartesian_cmd.twist.linear_y = 0.0
        self.cartesian_cmd.twist.linear_z = 0.0
        self.cartesian_cmd.twist.angular_x = 0.0
        self.cartesian_cmd.twist.angular_y = 0.0
        self.cartesian_cmd.twist.angular_z = 0.0
        self.cartesian_cmd.duration = 0

        self.joint_cmd = Base_JointSpeeds()
        for i in range(7):
            jspeed = JointSpeed()
            jspeed.joint_identifier = i
            jspeed.value = 0.0
            jspeed.duration = 0
            self.joint_cmd.joint_speeds.append(jspeed)
        self.joint_cmd.duration = 0

    def send_position_service_callback(self, req):
        print(req.position)
        file = position_files[req.position]
        with open(file) as fp:
            position = json.load(fp)
        success = self.send_joint_angles(position["value"])
        return sendRobotToPositionResponse(success)

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if self.last_action_notif_type == ActionEvent.ACTION_END:
                rospy.loginfo("Received ACTION_END notification")
                self.last_action_notif_type = None
                return True
            elif self.last_action_notif_type == ActionEvent.ACTION_ABORT:
                rospy.loginfo("Received ACTION_ABORT notification")
                self.last_action_notif_type = None
                return False
            else:
                # rospy.loginfo("waiting...")
                time.sleep(0.01)

    def send_cartesian_pose(self, pose, frame):
        req = ExecuteActionRequest()

        trajectory = WaypointList()
        waypoint = Waypoint()
        cartesianWaypoint = CartesianWaypoint()

        cartesianWaypoint.pose.x = pose[0]
        cartesianWaypoint.pose.y = pose[1]
        cartesianWaypoint.pose.z = pose[2]
        cartesianWaypoint.pose.theta_x = pose[3]
        cartesianWaypoint.pose.theta_y = pose[4]
        cartesianWaypoint.pose.theta_z = pose[5]

        blending_radius = 0
        if frame == "base":
            cartesianWaypoint.reference_frame = (
                CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_BASE
            )
        elif frame == "tool":
            cartesianWaypoint.reference_frame = (
                CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_TOOL
            )
        cartesianWaypoint.blending_radius = blending_radius

        # Initialize Waypoint and WaypointList
        waypoint.oneof_type_of_waypoint.cartesian_waypoint.append(cartesianWaypoint)
        trajectory.duration = 10
        trajectory.use_optimal_blending = True
        trajectory.waypoints.append(waypoint)
        try:
            res = self.validate_waypoint_list(trajectory)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ValidateWaypointList")
            return False
        error_number = len(res.output.trajectory_error_report.trajectory_error_elements)

        if error_number >= 1:
            rospy.loginfo("WaypointList is invalid")
            return False

        req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)

        # Send the cartesian pose
        rospy.loginfo("Sending the robot somewhere...")
        self.last_action_notif_type = None
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ExecuteWaypointjectory")
            return False
        else:
            return self.wait_for_action_end_or_abort()

    def send_joint_angles(self, joint_angles):
        req = ExecuteActionRequest()

        trajectory = WaypointList()
        waypoint = Waypoint()
        angularWaypoint = AngularWaypoint()

        # Angles to send the arm to vertical position (all zeros)
        for i in joint_angles:
            angularWaypoint.angles.append(i)

        # Each AngularWaypoint needs a duration and the global duration (from WaypointList) is disregarded.
        # If you put something too small (for either global duration or AngularWaypoint duration), the trajectory will be rejected.
        angular_duration = 5
        angularWaypoint.duration = angular_duration

        # Initialize Waypoint and WaypointList
        waypoint.oneof_type_of_waypoint.angular_waypoint.append(angularWaypoint)
        trajectory.duration = 5
        trajectory.use_optimal_blending = False
        trajectory.waypoints.append(waypoint)

        try:
            res = self.validate_waypoint_list(trajectory)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ValidateWaypointList")
            return False

        error_number = len(res.output.trajectory_error_report.trajectory_error_elements)
        MAX_ANGULAR_DURATION = 30

        while (error_number >= 1 and angular_duration <= MAX_ANGULAR_DURATION) :
            angular_duration += 1
            trajectory.waypoints[0].oneof_type_of_waypoint.angular_waypoint[
                0
            ].duration = angular_duration

            try:
                res = self.validate_waypoint_list(trajectory)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ValidateWaypointList")
                return False

            error_number = len(
                res.output.trajectory_error_report.trajectory_error_elements
            )

        if (angular_duration > MAX_ANGULAR_DURATION) :
            # It should be possible to reach position within 30s
            # WaypointList is invalid (other error than angularWaypoint duration)
            rospy.loginfo("WaypointList is invalid")
            return False

        req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)
        # Send the angles
        rospy.loginfo("Sending the robot somewhere...")
        self.last_action_notif_type = None
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ExecuteWaypointjectory")
            return False
        else:
            return self.wait_for_action_end_or_abort()

    def move_gripper(self, position):
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        if position[0] > MAX_FINGER_POS:
            finger.value = MAX_FINGER_POS
        elif position[0] < 0:
            finger.value = 0.0
        else:
            finger.value = position[0]
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION
        self.last_action_notif_type = None
        try:
            self.send_gripper_command(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand.")
            return False
        else:
            time.sleep(0.5)
            return True

    def execute_sequence(self, sequence_name):
        file = sequence_files[sequence_name]
        success = True
        with open(file) as fp:
            sequence = json.load(fp)

        for i, position in enumerate(sequence["positions"]):
            if success:
                if position["type"] == "joints":
                    if position["reference"] == "relative":
                        success = self.send_joint_angles(self.current_joint_angles + position["value"])
                    elif position["reference"] == "absolute":
                        success = self.send_joint_angles(position["value"])

                elif position["type"] == "cartesian":
                    if position["reference"] == "relative":
                        success = self.move_cartesian(position["value"], 'tool')
                    elif position["reference"] == "absolute":
                        success = self.move_cartesian(position["value"], 'base')
        
                elif position["type"] == "gripper":
                    if position["reference"] == "relative":
                        rospy.logerr("Relative position not supported in gripper mode.")
                    elif position["reference"] == "absolute":
                        success = self.move_gripper(position["value"])
            if success:
                self.sequence_idx_pub.publish(i)
                print(f"Position {i} achieved.")
            else:
                print("Failed to complete sequence")
                return False
            #time.sleep(0.5)
        return True

    def joy_callback(self, msg):
        # Cartesian control + gripper + sequences/predefined poses
        if msg.buttons[4]:
            self.cartesian_deadman = True
            self.joint_deadman = False
            # Arm control joysticks
            self.cartesian_cmd.twist.linear_x = msg.axes[0] * self.cartesian_speed_ratio
            self.cartesian_cmd.twist.linear_y = msg.axes[7] * self.cartesian_speed_ratio
            self.cartesian_cmd.twist.linear_z = msg.axes[1] * self.cartesian_speed_ratio
            self.cartesian_cmd.twist.angular_x = (
                msg.axes[4] * self.cartesian_speed_ratio
            )
            self.cartesian_cmd.twist.angular_y = (
                msg.axes[3] * self.cartesian_speed_ratio
            )
            self.cartesian_cmd.twist.angular_z = -msg.axes[6]

            # Fingers control
            if msg.axes[5] < 0:
                req = SendGripperCommandRequest()
                finger = Finger()
                finger.finger_identifier = 0
                finger.value = 1.0
                req.input.gripper.finger.append(finger)
                req.input.mode = GripperMode.GRIPPER_SPEED

                try:
                    self.send_gripper_command(req)
                except rospy.ServiceException:
                    rospy.logerr("Failed to call SendGripperCommand.")
                else:
                    self.fingers_moving = True
            elif msg.axes[2] < 0:
                req = SendGripperCommandRequest()
                finger = Finger()
                finger.finger_identifier = 0
                finger.value = -1.0
                req.input.gripper.finger.append(finger)
                req.input.mode = GripperMode.GRIPPER_SPEED

                try:
                    self.send_gripper_command(req)
                except rospy.ServiceException:
                    rospy.logerr("Failed to call SendGripperCommand.")
                else:
                    self.fingers_moving = True
            elif self.fingers_moving:
                req = SendGripperCommandRequest()
                finger = Finger()
                finger.finger_identifier = 0
                finger.value = 0.0
                req.input.gripper.finger.append(finger)
                req.input.mode = GripperMode.GRIPPER_SPEED

                try:
                    self.send_gripper_command(req)
                except rospy.ServiceException:
                    rospy.logerr("Failed to call SendGripperCommand.")
                else:
                    self.fingers_moving = False

            # Home position
            if msg.buttons[7]:
                self.cartesian_deadman = False
                rospy.loginfo("Homing")
                self.send_joint_angles(home_joint_angles)
            # Clear faults
            elif msg.buttons[6]:
                try:
                    self.clear_faults()
                except rospy.ServiceException:
                    rospy.logerr("Failed to call ClearFaults.")
                else:
                    rospy.loginfo("Cleared the faults succesfully.")

            elif msg.buttons[3]:  # Y
                self.cartesian_deadman = False
                rospy.loginfo("Moving Somewhere")
                self.send_cartesian_pose(test_cartesian_pose, "tool")
            elif msg.buttons[0]:  # A
                self.cartesian_deadman = False
                rospy.loginfo("Starting panoramic sequence")
                self.execute_sequence("panoramic_image")
            elif msg.buttons[1]:  # B
                self.cartesian_deadman = False
                _ = self.send_joint_angles(stair_test_pose)
            # elif msg.buttons[2]:    # X
            #     _ = self.execute_sequence("grab_dumbbell")
            # elif msg.buttons[2]:
            #     self.deadman = False
            #     self.send_joint_angles(test_joint_pose)
            # Retract pose, can shut down after and it won't fall
            if msg.buttons[8]:  # power
                self.cartesian_deadman = False
                self.send_joint_angles(retract_pose_joint_angles)

        # Joint control
        elif msg.buttons[5]:
            self.joint_deadman = True
            self.cartesian_deadman = False
            # print(" Joint positions : {}\n  Cartesian pose : {}\nFinger positions : {}\n".format(self.current_joint_angles, self.current_cartesian_pose, self.current_finger_positions))
            self.joint_cmd = Base_JointSpeeds()
            jspeed = JointSpeed()
            jspeed.duration = 0
            jspeed.joint_identifier = 0
            jspeed.value = msg.axes[0] * self.joint_speed_ratio
            self.joint_cmd.joint_speeds.append(jspeed)
            jspeed1 = JointSpeed()
            jspeed1.duration = 0
            jspeed1.joint_identifier = 1
            jspeed1.value = msg.axes[1] * self.joint_speed_ratio
            self.joint_cmd.joint_speeds.append(jspeed1)
            jspeed2 = JointSpeed()
            jspeed2.duration = 0
            jspeed2.joint_identifier = 2
            jspeed2.value = msg.axes[3] * self.joint_speed_ratio
            self.joint_cmd.joint_speeds.append(jspeed2)
            jspeed3 = JointSpeed()
            jspeed3.duration = 0
            jspeed3.joint_identifier = 3
            jspeed3.value = msg.axes[4] * self.joint_speed_ratio
            self.joint_cmd.joint_speeds.append(jspeed3)
            jspeed4 = JointSpeed()
            jspeed4.duration = 0
            jspeed4.joint_identifier = 4
            jspeed4.value = msg.axes[6] * self.joint_speed_ratio
            self.joint_cmd.joint_speeds.append(jspeed4)
            jspeed5 = JointSpeed()
            jspeed5.duration = 0
            jspeed5.joint_identifier = 5
            jspeed5.value = msg.axes[7] * self.joint_speed_ratio
            self.joint_cmd.joint_speeds.append(jspeed5)
        else:
            if self.joint_deadman or self.cartesian_deadman:
                self.init_cmd()
                self.rate.sleep()
                self.joint_deadman = False
                self.cartesian_deadman = False

    def base_feedback_callback(self, msg):
        self.current_cartesian_pose = np.array(
            [
                msg.base.commanded_tool_pose_x,
                msg.base.commanded_tool_pose_y,
                msg.base.commanded_tool_pose_z,
                msg.base.commanded_tool_pose_theta_x,
                msg.base.commanded_tool_pose_theta_y,
                msg.base.commanded_tool_pose_theta_z,
            ]
        )

    def joint_state_callback(self, msg):
        self.current_joint_angles = np.array(
            [
                degrees(msg.position[0]),
                degrees(msg.position[1]),
                degrees(msg.position[2]),
                degrees(msg.position[3]),
                degrees(msg.position[4]),
                degrees(msg.position[5]),
                degrees(msg.position[6]),
            ]
        )
        self.current_finger_positions = np.array([msg.position[7]])

    def dyn_reconfigure_callback(self, config, level):
        if config.cartesian_speed_ratio != self.cartesian_speed_ratio:
            self.cartesian_speed_ratio = config.cartesian_speed_ratio
        if config.joint_speed_ratio != self.joint_speed_ratio:
            self.joint_speed_ratio = config.joint_speed_ratio
        return config


if __name__ == "__main__":
    try:
        node = ArmControlNode()
    except rospy.ROSInterruptException:
        pass
