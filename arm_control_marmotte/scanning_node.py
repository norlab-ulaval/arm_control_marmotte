import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand
from control_msgs.msg import GripperCommand as GripperCommandMsg
from controller_manager_msgs.srv import SwitchController

# Joint points declaration
home_joint_point = JointTrajectoryPoint()
home_joint_point.positions = [0.0, 0.262, -3.14159, -2.269, 0.0, 0.96, 1.571]
home_joint_point.time_from_start.sec = 5
viewpoint_joint_point = JointTrajectoryPoint()
viewpoint_joint_point.positions = [0.0, -1.4, -3.14, -2.44, 0.0, -0.49, 1.568]
viewpoint_joint_point.time_from_start.sec = 5
joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']

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


class ScanningNode(Node):

    def __init__(self):
        super().__init__('scanning_node')
        np.set_printoptions(precision=4, suppress=True)

        # Parameters
        joint_topic = self.declare_parameter("joint_topic", "joint_trajectory_controller/joint_trajectory").value
        joy_topic = self.declare_parameter("joy_topic", "joy").value
        self.home_button = self.declare_parameter("home_button", 7).value
        self.viewpoint_button = self.declare_parameter("viewpoint_button", 6).value
        self.angle_step = self.declare_parameter("angle_step", 1.047).value

        # Messages
        self.joint_msg = JointTrajectory()
        self.joint_msg.joint_names = joint_names

        # Publishers/Subscribers
        self.joint_pub_ = self.create_publisher(JointTrajectory, joint_topic, 10)
        self.joy_sub = self.create_subscription(Joy, joy_topic, self.joy_callback, 10)

        self.home_arm()

    def home_arm(self):
        self.joint_msg.points = [home_joint_point]
        self.publish_joint_command()

    def move_to_viewpoint(self):
        self.joint_msg.points = [viewpoint_joint_point]
        self.publish_joint_command()

    def rotate_camera(self, direction):
        print(self.joint_msg.points)
        if direction == "left":
            self.joint_msg.points[0].positions[0] += self.angle_step
        elif direction == "right":
            self.joint_msg.points[0].positions[0] -= self.angle_step
        self.publish_joint_command()

    def publish_joint_command(self):
        self.joint_pub_.publish(self.joint_msg)

    def joy_callback(self, msg):
        # Start Button (for homing)
        if msg.buttons[self.home_button]:
            self.home_arm()

        # Back button (for viewpoint)
        elif msg.buttons[self.viewpoint_button]:
            self.move_to_viewpoint()

        elif msg.axes[6] < -0.5:
            self.rotate_camera("right")

        elif msg.axes[6] > 0.5:
            self.rotate_camera("left")


def main(args=None):
    rclpy.init(args=args)

    arm_control_node = ScanningNode()

    rclpy.spin(arm_control_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    arm_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
