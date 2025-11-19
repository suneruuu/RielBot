import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    ld = LaunchDescription()

    description_pkg_path = get_package_share_directory("rielbot_description")
    controller_pkg_path = get_package_share_directory("rielbot_controller")

    # Launch rviz2 from pck description
    display = IncludeLaunchDescription(
        os.path.join(description_pkg_path, "launch", "display.launch.py")
    )
    
    # Launch gazebo from pck description
    gazebo = IncludeLaunchDescription(
        os.path.join(description_pkg_path, "launch", "gazebo.launch.py")
    )

    controller = IncludeLaunchDescription(
        os.path.join(controller_pkg_path, "launch", "controller.launch.py")
    )

    # Launch the controller manager 3s after gazebo, to make sure the robot has spawned in simulation
    controller_delayed = TimerAction(
        period = 3., 
        actions=[controller]
    )
    
    # Launch the teleop keyboard node
    teleopkeyboard = IncludeLaunchDescription(
        os.path.join(controller_pkg_path,"launch","teleopkeyboard.launch.py"),
        launch_arguments={"use_sim_time": "True"}.items()
    )

    ld.add_action(display)
    ld.add_action(gazebo)
    ld.add_action(controller_delayed)
    ld.add_action(teleopkeyboard)

    return ld