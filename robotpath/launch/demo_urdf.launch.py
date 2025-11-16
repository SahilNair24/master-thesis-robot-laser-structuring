from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Path to the existing demo.launch.py inside abb_irb6660_moveit_config package
    demo_launch_path = os.path.join(
        get_package_share_directory("abb_irb6660_moveit_config"),
        "launch",
        "demo.launch.py"
    )

    # Include the existing demo launch
    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(demo_launch_path)
    )

    # Path to your workpiece URDF
    workpiece_urdf_path = os.path.join(
        get_package_share_directory("robotpath"),
        "urdf",
        "workpiece.urdf"
    )

    # Read the URDF content
    with open(workpiece_urdf_path, 'r') as f:
        workpiece_urdf = f.read()

    # Robot State Publisher node for workpiece
    workpiece_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="workpiece",
        name="workpiece_state_publisher",
        output="screen",
        parameters=[{"robot_description": workpiece_urdf}]
    )

    return LaunchDescription([
        demo_launch,
        workpiece_state_publisher
    ])
