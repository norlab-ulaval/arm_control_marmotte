import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node


def generate_launch_description():

    share_folder = get_package_share_directory("arm_control_marmotte")

    # Arm Control
    arm_config = os.path.join(share_folder, "scanning_node.yaml")
    scanning_node = Node(
        package="arm_control_marmotte",
        executable="scanning_node",
        name="scanning_node",
        parameters=[arm_config]
    )
    
    # Kortex Driver
    kortex_launch_file = os.path.join(get_package_share_directory('kortex_bringup'), 'launch', 'gen3.launch.py')
    kortex_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([kortex_launch_file]),
        launch_arguments={
            "robot_type": "gen3",
            "robot_ip": "192.168.5.111",
            "dof": "7",
            "use_fake_hardware": "false",
            "launch_rviz": "false",
        }.items()
    )

    # Kinova Gen3 camera
    vision_launch_file = os.path.join(get_package_share_directory('kinova_vision'), 'launch', 'kinova_vision.launch.py')
    kinova_vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([vision_launch_file]),
        launch_arguments={
            "camera": "gen3/camera",
            "device": "192.168.5.111",
            "camera_link_frame_id": "camera_link",
            "color_frame_id": "camera_color_frame",
            "depth_frame_id": "camera_depth_frame",
            "color_camera_info_url": "",
            "depth_camera_info_url": "",
            "launch_color": "true",
            "launch_depth": "false",
            "max_color_pub_rate": "15.0",
            "max_depth_pub_rate": "15.0"
        }.items()
    )
    
    return LaunchDescription([
        kortex_driver,
        kinova_vision,
        scanning_node
    ])