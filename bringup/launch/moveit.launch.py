# MoveIt Launch File for my_robot
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os
import yaml


def load_yaml(package_name, file_path):
    package_share = FindPackageShare(package_name).find(package_name)
    absolute_file_path = os.path.join(package_share, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        print(f"Error loading {absolute_file_path}: {e}")
        return {}


def generate_launch_description():
    pkg_share = FindPackageShare("my_robot_ros2_control")

    # --- Robot Description (URDF) ---
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([pkg_share, "description/urdf", "my_robot.urdf.xacro"]),
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # --- SRDF ---
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([pkg_share, "config/moveit", "my_robot.srdf"]),
    ])
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)
    }

    # --- Kinematics ---
    kinematics_yaml = load_yaml("my_robot_ros2_control", "config/moveit/kinematics.yaml")

    # --- Joint Limits ---
    joint_limits_yaml = load_yaml("my_robot_ros2_control", "config/moveit/joint_limits.yaml")
    joint_limits = {"robot_description_planning": joint_limits_yaml}

    # --- MoveIt Controllers ---
    moveit_controllers_yaml = load_yaml("my_robot_ros2_control", "config/moveit/moveit_controllers.yaml")

    # --- Planning Pipeline (Humble: pipeline 이름을 key로 감싸야 함) ---
    planning_pipeline = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization "
                                "default_planner_request_adapters/FixWorkspaceBounds "
                                "default_planner_request_adapters/FixStartStateBounds "
                                "default_planner_request_adapters/FixStartStateCollision "
                                "default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        }
    }

    # --- Trajectory Execution ---
    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 2.0,
        "trajectory_execution.allowed_goal_duration_margin": 1.0,
        "trajectory_execution.allowed_start_tolerance": 0.5,
        "trajectory_execution.execution_duration_monitoring": False,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # --- Move Group Node ---
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits,
            planning_pipeline,
            trajectory_execution,
            moveit_controllers_yaml,
            planning_scene_monitor_parameters,
        ],
    )

    return LaunchDescription([
        move_group_node,
    ])
