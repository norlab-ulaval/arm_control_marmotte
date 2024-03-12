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
home_joint_point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
home_joint_point.time_from_start.sec = 10
viewpoint_joint_point = JointTrajectoryPoint()
viewpoint_joint_point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
viewpoint_joint_point.time_from_start.sec = 10
joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']

# Gripper Commands declaration
open_gripper = GripperCommandMsg()
open_gripper.command.position = 0.0
open_gripper.command.max_effort = 100.0

close_gripper = GripperCommandMsg()
close_gripper.command.position = 0.8
close_gripper.command.max_effort = 100.0

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


class ArmControlNode(Node):

    def __init__(self):
        super().__init__('arm_control_node')
        np.set_printoptions(precision=4, suppress=True)

        # Publishers/Subscribers
        self.cmd_pub_ = self.create_publisher(Twist, 'twist_cmd', 10)
        self.joint_pub_ = self.create_publisher(JointTrajectory, 'joint_cmd', 10)
        self.speed_ratio = self.declare_parameter('speed_ratio', 1.0).value
        self.joy_sub = self.create_subscription(Joy, 'joy_topic', self.joy_callback, 10)

        # Actions
        self.gripper_action_client = ActionClient(self, GripperCommand, '/robotiq_gripper_controller/gripper_cmd')
        self.gripper_action_complete = False

        # Services
        self.switch_controller_client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        while not self.switch_controller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.switch_controller_request = SwitchController.Request()
        self.current_controller = "twist_controller"

        # Messages
        self.twist_msg = Twist()
        self.joint_msg = JointTrajectory()
        self.init_cmd()

        # timer to publish the commands.
        timer_period = 0.001  # seconds
        self.twist_pub_timer = self.create_timer(timer_period, self.twist_pub_callback)
        self.joint_pub_timer = self.create_timer(timer_period, self.joint_pub_callback)

    def init_cmd(self):
        self.twist_msg.linear.x = 0.0
        self.twist_msg.linear.y = 0.0
        self.twist_msg.linear.z = 0.0
        self.twist_msg.angular.x = 0.0
        self.twist_msg.angular.y = 0.0
        self.twist_msg.angular.z = 0.0

        self.joint_msg.joint_names = joint_names
        self.joint_msg.points = [home_joint_point]

    def send_gripper_goal(self, position):
        if position == 'open':
            goal_msg = open_gripper
        elif position == 'close':
            goal_msg = close_gripper
        else:
            return
        self.gripper_action_client.wait_for_server()
        self.send_gripper_goal_future = self.gripper_action_client.send_goal_async(goal_msg)
        self.send_gripper_goal_future.add_done_callback(self.send_gripper_goal_done_callback)

    def send_gripper_goal_done_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.gripper_action_complete = True
            return

        self.get_logger().info('Goal accepted :)')

        self.get_gripper_result_future = goal_handle.get_result_async()
        self.get_gripper_result_future.add_done_callback(self.get_gripper_result_callback)

    def get_gripper_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(result)
        self.gripper_action_complete = True


    def twist_pub_callback(self):
        self.cmd_pub_.publish(self.twist_msg)

    def joint_pub_callback(self):
        self.joint_pub_.publish(self.joint_msg)

    def switch_controller(self, start, stop):
        self.switch_controller_request.start_controllers = start
        self.switch_controller_request.stop_controllers = stop
        self.switch_controller_request.strictness = 2
        self.switch_controller_request.start_asap = True
        service_response = self.switch_controller_client.call_async(self.switch_controller_request)
        while not service_response.done():
            time.sleep(0.5)
            self.get_logger().info('Waiting for service completion')
        self.current_controller = start
        return service_response.result()

    def joy_callback(self, msg):
        self.get_logger().info('joy callback called')
        # Deadman switch
        if msg.buttons[4]:
            # Arm control joysticks
            self.twist_msg.linear.x= msg.axes[7]*self.speed_ratio
            self.twist_msg.linear.y = msg.axes[6]*self.speed_ratio
            self.twist_msg.linear.z = msg.axes[1]*self.speed_ratio
            self.twist_msg.angular.x = msg.axes[4]*self.speed_ratio
            self.twist_msg.angular.y = msg.axes[3]*self.speed_ratio
            self.twist_msg.angular.z = -msg.axes[0]*self.speed_ratio

            if msg.axes[5] < 0:
                self.gripper_action_complete = False
                self.send_gripper_goal('close')
                while not self.gripper_action_complete:
                    time.sleep(0.1)

            elif msg.axes[2] < 0:
                self.gripper_action_complete = False
                self.send_gripper_goal('open')
                while not self.gripper_action_complete:
                    time.sleep(0.1)

            elif msg.buttons[7]:
                self.joint_msg.joint_names = joint_names
                self.joint_msg.points = home_joint_point
                if self.current_controller == "twist_controller":
                    result = self.switch_controller('joint_trajectory_controller', 'twist_controller')
                    self.get_logger().info(result)

            elif msg.buttons[3]:
                self.joint_msg.joint_names = joint_names
                self.joint_msg.points = viewpoint_joint_point
                if self.current_controller == "twist_controller":
                    result = self.switch_controller('joint_trajectory_controller', 'twist_controller')
                    self.get_logger().info(result)

            else:
                if self.current_controller == "joint_trajectory_controller":
                    result = self.switch_controller('twist_controller', 'joint_trajectory_controller')
                    self.get_logger().info(result)


def main(args=None):
    rclpy.init(args=args)

    arm_control_node = ArmControlNode()

    rclpy.spin(arm_control_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    arm_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
