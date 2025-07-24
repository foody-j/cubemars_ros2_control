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
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # --- 런치 인자 선언 ---
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="RViz2를 자동으로 시작할지 결정합니다.",
        )
    )

    # --- 설정값 초기화 ---
    gui = LaunchConfiguration("gui")
    pkg_share = FindPackageShare("my_robot_ros2_control")

    # --- 파일 경로 설정 ---
    # URDF 파일을 xacro를 통해 로드합니다.
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([pkg_share, "description/urdf", "my_robot.urdf.xacro"]),
        ]
    )
    # ParameterValue로 감싸서 타입을 명확히 해줍니다. (Jazzy 호환성)
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # 컨트롤러 설정 파일을 로드합니다.
    robot_controllers = PathJoinSubstitution(
        [pkg_share, "bringup/config", "my_robot_controllers.yaml"]
    )

    # RViz 설정 파일을 로드합니다.
    rviz_config_file = PathJoinSubstitution([pkg_share, "rviz", "my_robot.rviz"])

    # --- 노드 정의 ---

    # 1. Controller Manager (ros2_control의 핵심)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="screen",
    )

    # 2. Robot State Publisher (URDF 기반으로 로봇의 TF를 발행)
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # 3. RViz (시각화 도구)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    # 4. Joint State Broadcaster Spawner (컨트롤러를 Controller Manager에 로드)
    #    - 로봇의 현재 상태(/joint_states)를 발행하는 역할을 합니다.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # 5. Joint Trajectory Controller Spawner (컨트롤러를 Controller Manager에 로드)
    #    - MoveIt이 보낸 궤적 명령을 실행하는 핵심 컨트롤러입니다.
    #    - joint_state_broadcaster가 성공적으로 로드된 후에 실행되도록 순서를 제어합니다.
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["my_robot_arm_controller", "--controller-manager", "/controller_manager"],
    )

    # --- 실행 순서 제어 ---
    # joint_state_broadcaster_spawner가 종료된 후(성공적으로 로드된 후) robot_controller_spawner를 실행합니다.
    # 이는 ros2_control 시스템의 안정적인 시작을 보장합니다.
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    nodes_to_start = [
        control_node,
        robot_state_pub_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
