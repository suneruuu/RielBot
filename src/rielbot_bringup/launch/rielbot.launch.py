import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # talker_node = Node (
    #     package = "demo_nodes_cpp",
    #     executable = "talker",
    #     name = "my_talker"
    # )

    # listener_node = Node (
    #     package = "demo_nodes_py",
    #     executable = "listener"
    # )

    # ld.add_action(talker_node)
    # ld.add_action(listener_node)

    description_pkg_path = get_package_share_directory("rielbot_description")
    # Launch rviz2 from pck description
    display = IncludeLaunchDescription(
        os.path.join(description_pkg_path, "launch", "display.launch.py")
    )
    
    # Launch gazebo from pck description
    gazebo = IncludeLaunchDescription(
        os.path.join(description_pkg_path, "launch", "gazebo.launch.py")
    )

    ld.add_action(display)
    ld.add_action(gazebo)

    return ld