# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue  # 추가


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")

    # Get URDF via xacro
    robot_description_content = ParameterValue(  # ParameterValue로 래핑
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("sfbot_can"),
                        "description/urdf",
                        "my_robot.urdf.xacro",
                    ]
                ),
            ]
        ),
        value_type=str  # 문자열로 명시
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("sfbot_can"),
            "config",
            "my_robot_controllers.yaml",
        ]
    )
    # RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("sfbot_can"), "rviz", "my_robot.rviz"]
    )

    # 노드 정의
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers, robot_description],  # robot_description 추가
        output="screen",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    
    # Forward Position Controller - INACTIVE로 시작
    forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_position_controller", 
            "--controller-manager", "/controller_manager",
            "--inactive"  # 비활성 상태로 시작
        ],
        output="screen",
    )
    
    # Joint Trajectory Controller - ACTIVE로 시작 (기본값)
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller", 
            "--controller-manager", "/controller_manager"
            # --inactive 플래그를 제거하여 활성 상태로 시작
        ],
        output="screen",
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui)
    )

    # 실행 순서 제어 - joint_trajectory_controller를 우선적으로 활성화
    delay_forward_position_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_trajectory_controller_spawner],  # trajectory controller를 먼저 시작
        )
    )
    
    delay_forward_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_trajectory_controller_spawner,
            on_exit=[forward_position_controller_spawner],  # 그 다음에 forward controller (inactive)
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_forward_position_controller,  
        delay_forward_controller,  # 수정된 이름
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)