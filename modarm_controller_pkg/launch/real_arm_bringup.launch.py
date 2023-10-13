import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    controller = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("modarm_controller_pkg"),
                "launch",
                "controller.launch.py"
            ),
            launch_arguments={"is_sim": "False"}.items()
        )
    
    moveit = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("modpi_ik_pkg"),
                "launch",
                "real_arm.launch.py"
            ),
            launch_arguments={"is_sim": "False"}.items()
        )
       
    return LaunchDescription([
        controller,
        # moveit,
    ])