import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):

    namespace = LaunchConfiguration('oakd_ns').perform(context)

    share_folder = get_package_share_directory('arm_control_marmotte')
    config_file = os.path.join(share_folder, "config", "camera_nn.yaml")

    oakd_node = ComposableNodeContainer(
            name="depthai_container",
            namespace=namespace,
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="depthai_ros_driver",
                    plugin="depthai_ros_driver::Camera",
                    name=namespace,
                    parameters=[
                        config_file,
                        {"nn.i_nn_config_path": os.path.join(share_folder, "config", "yolov8_doorknob_detection.json")},
                    ],
                ),
                ComposableNode(
                    package="image_proc",
                    plugin="image_proc::RectifyNode",
                    name="rectify_color_node",
                    namespace=namespace,
                    remappings=[
                        ("image", "rgb/image_raw"),
                        ("camera_info", "rgb/camera_info"),
                        ("image_rect", "rgb/image_rect"),
                        ("image_rect/compressed", "rgb/image_rect/compressed"),
                        ("image_rect/compressedDepth", "rgb/image_rect/compressedDepth"),
                        ("image_rect/theora", "rgb/image_rect/theora"),
                    ],
                ),
                ComposableNode(
                    package="depthai_filters",
                    name="segmentation_overlay",
                    plugin="depthai_filters::SegmentationOverlay",
                    namespace=namespace,
                    parameters=[config_file]
                ),
            ],
            arguments=["--ros-args", "--log-level", "debug"],
            output="both",
        )

    return [
        oakd_node
    ]


def generate_launch_description():
    namespace_launch_arg = DeclareLaunchArgument('oakd_ns', default_value='oak')

    return LaunchDescription([
        namespace_launch_arg,
        OpaqueFunction(function=launch_setup)
    ])

    