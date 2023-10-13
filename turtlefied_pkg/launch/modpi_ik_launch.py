#! /usr/bin/env python3
import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path

def load_file(file_path):
    """Load the contents of a file into a string"""
    try:
        with open(file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        return None

def load_yaml(file_path):
    """Load a yaml file into a dictionary"""
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def run_xacro(xacro_file):
    """Run xacro and output a file in the same directory with the same name, w/o a .xacro suffix"""
    urdf_file, ext = os.path.splitext(xacro_file)
    if ext != '.xacro':
        raise RuntimeError(f'Input file to xacro must have a .xacro extension, got {xacro_file}')
    os.system(f'xacro {xacro_file} -o {urdf_file}')
    return urdf_file


def generate_launch_description():
    ik_pkg_name = "modpi_ik_pkg"
    robot_name = "turtlefied"
    xacro_file = os.path.join(get_package_share_directory("turtlefied_pkg"), "urdf/" + robot_name + "_sim.urdf.xacro")
    urdf_file = run_xacro(xacro_file)
    srdf_file = os.path.join(get_package_share_directory(ik_pkg_name), "config/" + robot_name + ".srdf")
    kinematics_file = os.path.join(get_package_share_directory(ik_pkg_name), "config/kinematics.yaml" )
    ompl_config_file = os.path.join(get_package_share_directory(ik_pkg_name), "config/ompl_planning.yaml" )
    moveit_controllers_file = os.path.join(get_package_share_directory(ik_pkg_name), "config/controllers.yaml" )

    robot_description = load_file(urdf_file)
    robot_description_semantic = load_file(srdf_file)
    kinematics_config = load_yaml(kinematics_file)
    ompl_config = load_yaml(ompl_config_file)

    moveit_controllers = {
        'moveit_simple_controller_manager' : load_yaml(moveit_controllers_file),
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
    }
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01
    }
    planning_scene_monitor_config = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True
        # adding the ff: https://github.com/ros-planning/moveit2_tutorials/issues/528
        ,'publish_robot_description': True
        ,'publish_robot_description_semantic':True
    }

    # MoveIt node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'robot_description_semantic': robot_description_semantic,
                'robot_description_kinematics': kinematics_config,
                'ompl': ompl_config,
                'planning_pipelines': ['ompl'],
                'use_sim_time': True,
            },
            moveit_controllers,
            trajectory_execution,
            planning_scene_monitor_config,
        ],
    )
    rviz = Node(
        name='rviz',
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=["-d", LaunchConfiguration('rviz_config')],
        parameters=[
            {
                'robot_description': robot_description,
                'robot_description_semantic': robot_description_semantic,
                'robot_description_kinematics': kinematics_config,
                'use_sim_time': True,
            }
        ],
    )
    return LaunchDescription([
        move_group_node,
        rviz,
        ]
    )