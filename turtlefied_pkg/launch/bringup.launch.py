import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, TimerAction
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    path_to_rviz_config = "/path/to/config/slam.rviz"
    path_to_xplor_config = "/home/user/my_ws/src/m_explore/config/params.yaml"


    navigation_ = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("turtlefied_pkg"),
            "launch",
            "t_simulation_slam_launch.py"
        ),
        launch_arguments={"use_rviz": "False"
                          ,"headless": "False"
                          }.items()
    )

    robot_arm_control_ = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("turtlefied_pkg"),
            "launch",
            "modpi_ik_launch.py"
        ),
        launch_arguments={"rviz_config" : path_to_rviz_config}.items()
    )

    object_detection_ = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("yolox_ros_py"),
            "yolox_nano_onnx_gazebo.launch.py"
        ),
    )

    move_arm_service_server_ = Node(
        package="modpi_ik_pkg",
        executable="move_arm",
        name="arm_service_server",
        output="screen"
    )

    exploration_server_ = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("m_explore"),
            "launch",
            "explore.launch.py"
        ),
    )

    #nav2 map_server but should launch with unconfigured state
    map_server_ = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
                {'yaml_filename': '/home/user/my_ws/StoreMap20230905221415.yaml'}
            ]
    )

    # Introduce a delay before launching subsequent nodes
    default_arm_position_ = TimerAction(
        period=15.0,
        actions=[move_arm_service_server_,
                LogInfo(msg="Setting robot arm to initial pose complete.")
        ]
    )
    exploration_ = TimerAction(
        period=37.0, #for some reason navigation stack took too long to bring up
        actions=[
            exploration_server_,
            LogInfo(msg="Start Exploration.")
        ]
    )

    computer_vision_ = TimerAction(
        period=40.0,
        actions=[
            object_detection_,
            LogInfo(msg="Start Object Detection.")
        ]
    )

    return LaunchDescription([
        navigation_,
        robot_arm_control_,
        default_arm_position_,
        exploration_,
        map_server_,
        computer_vision_
    ])