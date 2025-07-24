# cubemars_ros2_control
CubeMars ROS2 Control Hardware Interface

개요 cubemars_ros2_control은 CubeMars社의 서보 모터 시리즈(AK 시리즈)로 구성된 6축 다관절 로봇 팔을 위한 ros2_control 하드웨어 인터페이스 패키지입니다.
이 패키지는 CAN 통신을 통해 실제 하드웨어와 ROS 2 생태계를 연결하는 다리 역할을 하며, moveit2와 같은 상위 레벨의 모션 플래닝 프레임워크와 함께 사용되도록 설계되었습니다. 

주요 특징 ros2_control SystemInterface 구현: ros2_control 프레임워크의 표준을 준수하여 안정성과 확장성을 확보했습니다.
CAN 통신 기반: SocketCAN을 사용하여 CubeMars 모터와 직접 통신합니다.

위치 제어 인터페이스: 각 관절의 위치(Position) 명령을 받아 하드웨어에 전달하고, 현재 상태(Position, Velocity)를 ROS 2 시스템에 보고합니다.

다양한 모터 조합 지원: CubeMars AK60-84, AK70-10, AK60-6 등 여러 종류의 모터가 조합된 로봇 팔 구성을 지원합니다.

시스템 요구사항 OS: Ubuntu 24.04
ROS 버전: ROS 2 Jazzy

하드웨어:

CAN 통신 인터페이스 (예: CANable, PCAN-USB 등)

CubeMars AK 시리즈 모터로 구성된 6축 로봇 팔

설치 및 빌드 # 1. ROS 2 워크스페이스로 이동합니다. cd ~/ros2_ws/src
# 2. 레포지토리를 클론합니다.
git clone https://github.com/foody-j/cubemars_ros2_control

# 3. 워크스페이스 루트로 이동하여 의존성을 설치합니다.
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 4. 패키지를 빌드합니다.
colcon build --packages-select cubemars_ros2_control

사용법 이 패키지는 단독으로 실행되기보다는, 로봇의 URDF 파일 및 컨트롤러 설정과 함께 런치 파일을 통해 실행됩니다.
5.1. URDF 설정
로봇의 URDF 파일에 ros2_control 태그를 추가하여 이 하드웨어 인터페이스를 사용하도록 지정해야 합니다.

<!-- 예시: my_robot.urdf.xacro -->
<ros2_control name="MyRobotSystem" type="system">
    <hardware>
        <plugin>cubemars_ros2_control/MyRobotHardwareInterface</plugin>
        <!-- CAN 버스 인터페이스 이름 등 파라미터를 여기에 추가할 수 있습니다. -->
        <param name="can_interface_name">can0</param>
    </hardware>
    <joint name="link1_1_joint">
        <command_interface name="position"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
    </joint>
    <!-- ... 나머지 5개 관절에 대해서도 동일하게 설정 ... -->
</ros2_control>

5.2. 컨트롤러 설정 (controllers.yaml)
joint_trajectory_controller와 joint_state_broadcaster를 사용하도록 설정 파일을 작성합니다.

controller_manager:
  ros__parameters:
    update_rate: 100

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    my_robot_arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

my_robot_arm_controller:
  ros__parameters:
    joints:
      - link1_1_joint
      - link2_1_joint
      # ... 나머지 관절 이름 ...
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity

5.3. 런치 파일 실행
준비된 런치 파일을 통해 로봇의 모든 노드를 실행합니다.

# 워크스페이스를 source하는 것을 잊지 마세요.
source ~/ros2_ws/install/setup.bash

# 런치 파일 실행
ros2 launch cubemars_ros2_control my_robot.launch.py
