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
from sensor_msgs.msg import JointState
import math
from copy import deepcopy

# Joint points declaration
home_joint_point = JointTrajectoryPoint()
# home_joint_point.positions = [0.0, 0.262, -3.14159, -2.269, 0.0, 0.96, 1.571]
home_joint_point.positions = [0.0, -1.4, -3.14, -2.44, 0.0, -0.49, 1.568]
viewpoint_joint_point = JointTrajectoryPoint()
viewpoint_joint_point.positions = [0.0, -1.4, -3.14, -2.44, 0.0, -0.49, 1.568]

# fullscan_viewpoint_point = JointTrajectoryPoint()
# fullscan_viewpoint_point.positions = [0.0, 0.0, -3.14, -1.46, 0.0, -1.42, 1.57]
# joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']

fullscan_viewpoint_point = JointTrajectoryPoint()
fullscan_viewpoint_point.positions = [0.0, 0.0, -3.14, -1.46, 0.0, 1.4, 1.57]
joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']

# # noel
# fullscan_viewpoint_point = JointTrajectoryPoint()
# fullscan_viewpoint_point.positions = [0.0, 0.0, -3.14, -1.46, 0.0, 0.8, 1.57]
# joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']

'''
    Logitech Joystick Button Mapping

    axes[0] -> Left joystick (left=1/right=-1)
    axes[1] -> Left joystick (up=1/down=-1)
    axes[2] -> LT (default=1, when pressed up to -1)
    axes[3] -> Right joystick (left=1/right=-1)
    axes[4] -> Right joystick (up=1/down=-1)
    axes[5] -> RT (default=1, when pressed up to -1)
    axes[6] -> ArrowPad (Right=-1/Left=1)
    axes[7] -> ArrowPad (Down=-1/Up=1)

    buttons[0] -> A
    buttons[1] -> B
    buttons[2] -> X
    buttons[3] -> Y
    buttons[4] -> LB
    buttons[5] -> RB
    buttons[6] -> Back
    buttons[7] -> Home
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
        self.declare_parameter("home_button", 7)
        self.declare_parameter("viewpoint_button", 6)
        self.declare_parameter("grip_button", 1)
        self.declare_parameter("angle_step", 1.047)
        self.declare_parameter("angular_velocity", 1.0)
        self.declare_parameter("A_button", 0)

        self.update_parameters()

        # Messages
        self.joint_msg = JointTrajectory()
        self.joint_msg.joint_names = joint_names
        self.current_joint_positions = [0.0] * len(joint_names)

        # Publishers/Subscribers
        self.joint_pub_ = self.create_publisher(JointTrajectory, joint_topic, 10)
        self.joy_sub = self.create_subscription(Joy, joy_topic, self.joy_callback, 10)
        self.remote_joy_sub = self.create_subscription(Joy, "/remote_joy", self.joy_callback, 10)

        self.joint_state_sub = self.create_subscription(JointState, "joint_states", self.joint_state_callback, 10)  # Subscribe to joint states

        # Timer for dynamic parameters
        self.create_timer(2.0, self.update_parameters)

        # Debounce for buttons
        self.debounce_time = 0.2
        self.last_button_press = time.time()

        # Home the arm
        self.home_arm()

        self.turning_direction = -1
        self.ready_toScan = 0

    def home_arm(self):
        self.get_logger().info("Homing the arm...")
        self.joint_msg.points = [deepcopy(home_joint_point)]
        self.compute_trajectory_time(self.joint_msg.points[0])
        self.publish_joint_command()

    def move_to_viewpoint(self):
        self.get_logger().info("Moving to viewpoint...")
        self.joint_msg.points = [deepcopy(viewpoint_joint_point)]
        self.compute_trajectory_time(self.joint_msg.points[0])
        self.publish_joint_command()


    def scanning(self):
        time_step = 4

        self.get_logger().info("Scaning 360...")
        self.joint_msg.points = [deepcopy(viewpoint_joint_point)]
        self.joint_msg.points[0].time_from_start.sec = time_step
        
        
        for i in range(1, 7):
            new_point = JointTrajectoryPoint()
            time_step += 4

            new_point.positions = self.joint_msg.points[i-1].positions
            if self.turning_direction < -0.5:
                new_point.positions[0] += self.angle_step
            else:
                new_point.positions[0] -= self.angle_step

            self.joint_msg.points.append(deepcopy(new_point))
            self.joint_msg.points[i].time_from_start.sec = time_step

        self.publish_joint_command()
        self.turning_direction *= -1

    def grip(self):
        self.joint_msg.points[-1] = 1.0
        self.publish_joint_command()


    def rotate_camera(self, direction):
        

        self.get_logger().info(f"Up/Down Scaning...")

        new_point = deepcopy(fullscan_viewpoint_point)

        self.joint_msg.points = [deepcopy(new_point)]
        self.joint_msg.points[0].time_from_start.sec = 4

        self.get_logger().info(f"Direction: {direction}...")
        new_angle = 0 #self.current_joint_positions[0]
        if direction == "left":
            new_angle = -1.47
        elif direction == "right":
            new_angle =  1.47
        if direction == "front":
            new_angle = 0
        elif direction == "back":
            new_angle =  3.14

        arm_direction_point = JointTrajectoryPoint()
        arm_direction_point.positions = self.joint_msg.points[0].positions
        arm_direction_point.positions[0] = new_angle

        self.joint_msg.points.append(deepcopy(arm_direction_point))
        self.joint_msg.points[1].time_from_start.sec = 8

        camera_direction_point = JointTrajectoryPoint()
        camera_direction_point.positions = self.joint_msg.points[1].positions
        camera_direction_point.positions[5] = -1.4

        self.joint_msg.points.append(deepcopy(camera_direction_point))
        self.joint_msg.points[2].time_from_start.sec = 11

        self.publish_joint_command()

    def say_bye(self):

        time_step = 4

        self.get_logger().info("Saying bye...")
        self.joint_msg.points = [deepcopy(fullscan_viewpoint_point)]

        self.joint_msg.points[0].time_from_start.sec = time_step
        
        new_point = JointTrajectoryPoint()
        time_step += 4

        new_point.positions = self.joint_msg.points[0].positions
        new_point.positions[4] += self.angle_step

        self.joint_msg.points.append(deepcopy(new_point))
        self.joint_msg.points[1].time_from_start.sec = time_step


        new_point.positions = self.joint_msg.points[0].positions
        new_point.positions[4] -= (2*self.angle_step)

        self.joint_msg.points.append(deepcopy(new_point))
        self.joint_msg.points[2].time_from_start.sec = time_step + 4

        self.publish_joint_command()
  
    def step_rotate_camera(self, direction):
        self.get_logger().info(f"Step {direction}...")
        
        if direction == "left":
            new_angle = self.current_joint_positions[0]
            new_angle -= 0.3
            self.joint_msg.points[0].positions[0] = new_angle
        elif direction == "right":
            new_angle = self.current_joint_positions[0]
            new_angle += 0.3
            self.joint_msg.points[0].positions[0] = new_angle    
        elif direction == "up":
            new_angle = self.current_joint_positions[5]
            new_angle += 0.3
            self.joint_msg.points[0].positions[5] = new_angle
        elif direction == "down":
            new_angle = self.current_joint_positions[5]
            new_angle -= 0.3
            self.joint_msg.points[0].positions[5] = new_angle

        self.compute_trajectory_time(self.joint_msg.points[0])
        self.publish_joint_command()

    def compute_trajectory_time(self, points):
        max_diff = max(abs(self.wrap_to_pi(current - desired)) for current, desired in zip(self.current_joint_positions, points.positions))
        required_time = max_diff / self.angular_velocity
        points.time_from_start.sec = int(required_time) 
        points.time_from_start.nanosec = int((required_time - int(required_time)) * 1e9)
    

    def compute_trajectory_time_list(self, init_points, end_points):
        max_diff = max(abs(self.wrap_to_pi(current - desired)) for current, desired in zip(init_points, end_points))
        return  (max_diff / self.angular_velocity)
        

    def wrap_to_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def publish_joint_command(self):
        self.joint_pub_.publish(self.joint_msg)

    def joy_callback(self, msg):

        if time.time() - self.last_button_press < self.debounce_time:
            return

        # Start Button (for homing)
        if msg.buttons[self.home_button]:
            self.home_arm()

        # Back button (for viewpoint)
        elif msg.buttons[self.viewpoint_button]:
            self.scanning()
        elif msg.buttons[self.grip_button]:
            self.grip()

        # elif msg.axes[6] < -0.5:
        #     self.rotate_camera("right")

        # elif msg.axes[6] > 0.5:
        #     self.rotate_camera("left")
        
        # elif msg.axes[7] < -0.5:
        #     self.rotate_camera("back")

        # elif msg.axes[7] > 0.5:
        #     self.rotate_camera("front")
        
        # elif msg.axes[0] < -0.5:
        elif msg.axes[6] < -0.5:
            self.step_rotate_camera("right")

        # elif msg.axes[0] > 0.5:
        elif msg.axes[6] > 0.5:
            self.step_rotate_camera("left")
        
        # elif msg.axes[1] < -0.5:
        elif msg.axes[7] < -0.5:
            self.step_rotate_camera("down")

        # elif msg.axes[1] > 0.5:
        elif msg.axes[7] > 0.5:
            self.step_rotate_camera("up")
        
        # elif msg.buttons[self.A_button]:
        #     self.say_bye()

        self.last_button_press = time.time()

    def joint_state_callback(self, msg):
        # Update current joint positions with feedback from joint states
        for i, name in enumerate(self.joint_msg.joint_names):
            if name in msg.name:
                index = msg.name.index(name)
                self.current_joint_positions[i] = msg.position[index]

    def update_parameters(self):
        self.home_button = self.get_parameter("home_button").value
        self.viewpoint_button = self.get_parameter("viewpoint_button").value
        self.grip_button = self.get_parameter("grip_button").value
        self.angle_step = self.get_parameter("angle_step").value
        self.angular_velocity = self.get_parameter("angular_velocity").value
        self.A_button = self.get_parameter("A_button").value


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
