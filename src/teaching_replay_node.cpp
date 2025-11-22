/**
 * @file teaching_replay_node.cpp
 * @brief 교시 모드에서 기록된 로봇 동작을 재생하는 ROS2 노드
 * 
 * 이 노드는 teaching_data_logger.hpp에서 생성된 CSV 파일을 읽어서
 * JointTrajectory 메시지로 변환한 뒤, JointTrajectoryController에게 전달하여
 * 로봇이 교시받은 동작을 정확히 재현하도록 합니다.
 * 
 * @author Your Name
 * @date 2025
 */

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <memory>
#include <cmath>

// ============================================================================
// CSV에서 읽어온 한 프레임(시간 단위)의 교시 데이터를 담는 구조체
// ============================================================================
struct TeachingFrame {
    double timestamp_sec;                    // 해당 프레임의 시간(초 단위)
    std::vector<double> joint_positions;     // 6개 관절의 위치 (라디안)
    std::vector<double> joint_velocities;    // 6개 관절의 속도 (라디안/초)
    
    // 생성자: 기본적으로 6개 관절을 가정
    TeachingFrame(size_t num_joints = 6) 
        : timestamp_sec(0.0),
          joint_positions(num_joints, 0.0),
          joint_velocities(num_joints, 0.0) {}
};

// ============================================================================
// Teaching Replay 노드 클래스
// ============================================================================
class TeachingReplayNode : public rclcpp::Node {
public:
    /**
     * @brief 생성자 - 노드 초기화 및 파라미터/토픽/서비스 설정
     */
    TeachingReplayNode() : Node("teaching_replay_node") {
        
        // ========================================
        // 1. ROS2 파라미터 선언
        // ========================================
        // 사용자가 런치 파일이나 커맨드로 설정할 수 있는 파라미터들
        this->declare_parameter("csv_file", "");  // 교시 CSV 파일 경로
        this->declare_parameter("replay_speed", 1.0);  // 재생 속도 배율 (1.0 = 원속도)
        this->declare_parameter("loop", false);  // 반복 재생 여부 (현재 미구현)
        this->declare_parameter("controller_name", "my_robot_arm_controller");  // 컨트롤러 이름
        
        // ========================================
        // 2. Joint 이름 설정
        // ========================================
        // URDF에 정의된 관절 이름과 정확히 일치해야 함!
        joint_names_ = {
            "link1_1_joint",  // 관절 1
            "link2_1_joint",  // 관절 2
            "link3_1_joint",  // 관절 3
            "link4_1_joint",  // 관절 4
            "link5_1_joint",  // 관절 5
            "link6_1_joint"   // 관절 6
        };
        
        // ========================================
        // 3. Publisher 생성
        // ========================================
        // JointTrajectory 메시지를 JointTrajectoryController에게 전송하는 퍼블리셔
        // 토픽 이름: /<controller_name>/joint_trajectory
        std::string controller_name = this->get_parameter("controller_name").as_string();
        std::string trajectory_topic = "/" + controller_name + "/joint_trajectory";
        
        trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            trajectory_topic,
            10  // QoS: 메시지 큐 크기
        );
        
        // ========================================
        // 4. Service 서버 생성
        // ========================================
        // 재생 시작 서비스: ros2 service call /start_replay std_srvs/srv/Trigger
        start_service_ = this->create_service<std_srvs::srv::Trigger>(
            "start_replay",
            std::bind(&TeachingReplayNode::start_replay_callback, this,
                     std::placeholders::_1, std::placeholders::_2)
        );
        
        // 재생 중지 서비스: ros2 service call /stop_replay std_srvs/srv/Trigger
        stop_service_ = this->create_service<std_srvs::srv::Trigger>(
            "stop_replay",
            std::bind(&TeachingReplayNode::stop_replay_callback, this,
                     std::placeholders::_1, std::placeholders::_2)
        );
        
        // ========================================
        // 5. 초기화 완료 메시지
        // ========================================
        RCLCPP_INFO(this->get_logger(), "✅ Teaching Replay Node initialized");
        RCLCPP_INFO(this->get_logger(), "📡 Publishing to: %s", trajectory_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "💡 Use: ros2 service call /start_replay std_srvs/srv/Trigger");
    }
    
private:
    // ========================================
    // 멤버 변수
    // ========================================
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
    
    std::vector<std::string> joint_names_;       // 관절 이름 목록
    std::vector<TeachingFrame> teaching_frames_; // CSV에서 로드된 전체 프레임 데이터
    
    // ========================================
    // 핵심 함수 1: CSV 파일 로드
    // ========================================
    /**
     * @brief CSV 파일을 읽어서 teaching_frames_ 벡터에 저장
     * 
     * CSV 형식:
     * timestamp_ms,timestamp_sec,frame_count,
     * joint1_rad,joint2_rad,...,joint6_rad,
     * joint1_vel_rpm,joint2_vel_rpm,...,joint6_vel_rpm,
     * joint1_current_A,...,joint1_deg,...,ee_x_m,...
     * 
     * @param csv_file CSV 파일 경로
     * @return 성공 여부
     */
    bool load_teaching_data(const std::string& csv_file) {
        // 파일 열기
        std::ifstream file(csv_file);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to open CSV file: %s", csv_file.c_str());
            return false;
        }
        
        // 기존 데이터 초기화
        teaching_frames_.clear();
        
        std::string line;
        
        // ========================================
        // 첫 줄(헤더) 스킵
        // ========================================
        std::getline(file, line);
        
        int frame_count = 0;
        
        // ========================================
        // 각 줄을 읽어서 TeachingFrame으로 변환
        // ========================================
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string value;
            std::vector<std::string> row;
            
            // 쉼표로 분리하여 각 값을 벡터에 저장
            while (std::getline(ss, value, ',')) {
                row.push_back(value);
            }
            
            // CSV 형식 검증: 최소 15개 컬럼 필요
            // (timestamp_ms, timestamp_sec, frame_count, joint1~6_rad, joint1~6_vel_rpm)
            if (row.size() < 15) {
                RCLCPP_WARN(this->get_logger(), "Skipping incomplete row (size: %zu)", row.size());
                continue;
            }
            
            TeachingFrame frame;
            
            try {
                // ========================================
                // 1. Timestamp 추출 (초 단위) - 인덱스 1
                // ========================================
                frame.timestamp_sec = std::stod(row[1]);
                
                // ========================================
                // 2. Joint Positions 추출 (라디안) - 인덱스 3~8
                // ========================================
                // CSV: joint1_rad, joint2_rad, ..., joint6_rad
                for (int i = 0; i < 6; i++) {
                    frame.joint_positions[i] = std::stod(row[3 + i]);
                }
                
                // ========================================
                // 3. Joint Velocities 추출 및 변환 - 인덱스 9~14
                // ========================================
                // CSV에는 RPM 단위로 저장되어 있음
                // ROS2 Trajectory는 rad/s 단위를 사용하므로 변환 필요
                // 변환 공식: rad/s = RPM × (2π / 60)
                for (int i = 0; i < 6; i++) {
                    double rpm = std::stod(row[9 + i]);
                    frame.joint_velocities[i] = rpm * 2.0 * M_PI / 60.0;  // RPM → rad/s
                }
                
                // 프레임을 벡터에 추가
                teaching_frames_.push_back(frame);
                frame_count++;
                
            } catch (const std::exception& e) {
                // 숫자 변환 실패 등의 에러 발생 시 해당 줄 스킵
                RCLCPP_WARN(this->get_logger(), "Skipping invalid frame: %s", e.what());
                continue;
            }
        }
        
        file.close();
        
        RCLCPP_INFO(this->get_logger(), "✅ Loaded %d teaching frames from CSV", frame_count);
        
        // 프레임이 하나도 없으면 실패
        if (teaching_frames_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "❌ No valid frames found in CSV");
            return false;
        }
        
        // ========================================
        // 타임스탬프 정규화
        // ========================================
        // 첫 프레임의 시간을 0으로 만들어서 상대 시간으로 변환
        // 예: [5.0, 5.1, 5.2] → [0.0, 0.1, 0.2]
        double first_timestamp = teaching_frames_[0].timestamp_sec;
        for (auto& frame : teaching_frames_) {
            frame.timestamp_sec -= first_timestamp;
        }
        
        RCLCPP_INFO(this->get_logger(), "📊 Trajectory duration: %.2f seconds", 
                   teaching_frames_.back().timestamp_sec);
        
        return true;
    }
    
    // ========================================
    // 핵심 함수 2: Trajectory 생성 및 전송
    // ========================================
    /**
     * @brief 로드된 teaching_frames_를 JointTrajectory 메시지로 변환하여 전송
     * 
     * 동작 원리:
     * 1. 100Hz로 저장된 데이터를 다운샘플링 (성능 최적화)
     * 2. 각 프레임을 JointTrajectoryPoint로 변환
     * 3. replay_speed 파라미터로 속도 조절
     * 4. JointTrajectoryController에게 전송
     */
    void send_trajectory() {
        // 데이터가 없으면 중단
        if (teaching_frames_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "❌ No teaching data loaded!");
            return;
        }
        
        // ========================================
        // 1. Trajectory 메시지 생성
        // ========================================
        auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
        // 🔧 시작 시간에 0.5초 여유를 줘서 컨트롤러가 준비할 시간 확보
        trajectory_msg.header.stamp = this->now() + rclcpp::Duration::from_seconds(0.5);
        trajectory_msg.joint_names = joint_names_;  // 관절 이름 설정
        
        // ========================================
        // 2. 재생 속도 가져오기
        // ========================================
        // 예: 0.5 = 절반 속도, 2.0 = 2배 속도
        double replay_speed = this->get_parameter("replay_speed").as_double();
        
        // ========================================
        // 3. 다운샘플링 설정
        // ========================================
        // 100Hz 데이터를 그대로 보내면 너무 많은 포인트 → 성능 저하
        // sampling_rate=5: 5프레임마다 1개씩 샘플링 (100Hz → 20Hz)
        int sampling_rate = 5;
        
        RCLCPP_INFO(this->get_logger(), 
                   "🎬 Creating trajectory with %zu frames (sampled from %zu)...", 
                   teaching_frames_.size() / sampling_rate,
                   teaching_frames_.size());
        
        // ========================================
        // 4. 프레임을 Trajectory Point로 변환
        // ========================================
        for (size_t i = 0; i < teaching_frames_.size(); i += sampling_rate) {
            const auto& frame = teaching_frames_[i];
            
            // JointTrajectoryPoint 생성
            trajectory_msgs::msg::JointTrajectoryPoint point;
            
            // 위치 설정 (rad)
            point.positions = frame.joint_positions;
            
            // 속도 설정 (rad/s)
            point.velocities = frame.joint_velocities;
            
            // 시간 설정 (replay_speed로 조정)
            // 예: 원본 10초, replay_speed=0.5 → 20초에 걸쳐 재생
            point.time_from_start = rclcpp::Duration::from_seconds(
                frame.timestamp_sec / replay_speed
            );
            
            // Trajectory에 포인트 추가
            trajectory_msg.points.push_back(point);
        }
        
        // ========================================
        // 5. 마지막 프레임 보정
        // ========================================
        // 다운샘플링 때문에 마지막 프레임이 누락될 수 있음
        // 정확한 종료 지점을 위해 마지막 프레임을 반드시 포함
        size_t last_index = teaching_frames_.size() - 1;
        if (last_index % sampling_rate != 0) {
            const auto& last_frame = teaching_frames_[last_index];
            
            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions = last_frame.joint_positions;
            point.velocities = last_frame.joint_velocities;
            point.time_from_start = rclcpp::Duration::from_seconds(
                last_frame.timestamp_sec / replay_speed
            );
            
            trajectory_msg.points.push_back(point);
        }
        
        // ========================================
        // 6. Trajectory 전송
        // ========================================
        trajectory_pub_->publish(trajectory_msg);
        
        // 완료 로그 출력
        double total_duration = teaching_frames_.back().timestamp_sec / replay_speed;
        RCLCPP_INFO(this->get_logger(), 
                   "🚀 Trajectory sent! Duration: %.2f seconds, Points: %zu",
                   total_duration, 
                   trajectory_msg.points.size());
        
        RCLCPP_INFO(this->get_logger(), 
                   "⏱️  Replay speed: %.1fx (%.2fs → %.2fs)", 
                   replay_speed,
                   teaching_frames_.back().timestamp_sec,
                   total_duration);
    }
    
    // ========================================
    // Service 콜백 1: 재생 시작
    // ========================================
    /**
     * @brief /start_replay 서비스 콜백 함수
     * 
     * 사용법: ros2 service call /start_replay std_srvs/srv/Trigger
     */
    void start_replay_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        // request는 사용하지 않음 (Trigger 서비스는 빈 요청)
        (void)request;
        
        RCLCPP_INFO(this->get_logger(), "📞 Received start_replay service call");
        
        // ========================================
        // 1. CSV 파일 경로 확인
        // ========================================
        std::string csv_file = this->get_parameter("csv_file").as_string();
        
        if (csv_file.empty()) {
            response->success = false;
            response->message = "CSV file parameter not set! Use: ros2 param set /teaching_replay_node csv_file /path/to/file.csv";
            RCLCPP_ERROR(this->get_logger(), "❌ %s", response->message.c_str());
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "📂 Loading teaching data from: %s", csv_file.c_str());
        
        // ========================================
        // 2. CSV 파일 로드
        // ========================================
        if (!load_teaching_data(csv_file)) {
            response->success = false;
            response->message = "Failed to load CSV file. Check file path and format.";
            RCLCPP_ERROR(this->get_logger(), "❌ %s", response->message.c_str());
            return;
        }
        
        // ========================================
        // 3. Trajectory 전송
        // ========================================
        send_trajectory();
        
        // ========================================
        // 4. 성공 응답
        // ========================================
        response->success = true;
        response->message = "Replay started successfully!";
        RCLCPP_INFO(this->get_logger(), "✅ %s", response->message.c_str());
    }
    
    // ========================================
    // Service 콜백 2: 재생 중지
    // ========================================
    /**
     * @brief /stop_replay 서비스 콜백 함수
     * 
     * 빈 trajectory를 전송하여 현재 실행 중인 궤적을 취소합니다.
     * 
     * 사용법: ros2 service call /stop_replay std_srvs/srv/Trigger
     */
    void stop_replay_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        
        RCLCPP_INFO(this->get_logger(), "📞 Received stop_replay service call");
        
        // ========================================
        // 빈 Trajectory 전송 (현재 동작 취소)
        // ========================================
        auto empty_trajectory = trajectory_msgs::msg::JointTrajectory();
        empty_trajectory.header.stamp = this->now();
        empty_trajectory.joint_names = joint_names_;
        // points는 비어있음 → 컨트롤러가 현재 동작을 중지
        
        trajectory_pub_->publish(empty_trajectory);
        
        response->success = true;
        response->message = "Replay stopped";
        RCLCPP_INFO(this->get_logger(), "⏹️  %s", response->message.c_str());
    }
};

// ============================================================================
// main 함수
// ============================================================================
/**
 * @brief 프로그램 진입점
 * 
 * ROS2 노드를 초기화하고 spin하여 서비스 콜백을 대기합니다.
 */
int main(int argc, char** argv) {
    // ROS2 초기화
    rclcpp::init(argc, argv);
    
    // 노드 생성
    auto node = std::make_shared<TeachingReplayNode>();
    
    // ========================================
    // 사용법 안내
    // ========================================
    RCLCPP_INFO(node->get_logger(), "");
    RCLCPP_INFO(node->get_logger(), "🎮 Teaching Replay Node started!");
    RCLCPP_INFO(node->get_logger(), "");
    RCLCPP_INFO(node->get_logger(), "📋 Usage:");
    RCLCPP_INFO(node->get_logger(), "  1. Set CSV file parameter:");
    RCLCPP_INFO(node->get_logger(), "     ros2 param set /teaching_replay_node csv_file /path/to/teaching.csv");
    RCLCPP_INFO(node->get_logger(), "");
    RCLCPP_INFO(node->get_logger(), "  2. Adjust replay speed (optional):");
    RCLCPP_INFO(node->get_logger(), "     ros2 param set /teaching_replay_node replay_speed 0.5  # 50%% speed");
    RCLCPP_INFO(node->get_logger(), "");
    RCLCPP_INFO(node->get_logger(), "  3. Start replay:");
    RCLCPP_INFO(node->get_logger(), "     ros2 service call /start_replay std_srvs/srv/Trigger");
    RCLCPP_INFO(node->get_logger(), "");
    RCLCPP_INFO(node->get_logger(), "  4. Stop replay:");
    RCLCPP_INFO(node->get_logger(), "     ros2 service call /stop_replay std_srvs/srv/Trigger");
    RCLCPP_INFO(node->get_logger(), "");
    RCLCPP_INFO(node->get_logger(), "⏳ Waiting for service calls...");
    RCLCPP_INFO(node->get_logger(), "");
    
    // ========================================
    // 노드 실행 (서비스 콜백 대기)
    // ========================================
    rclcpp::spin(node);
    
    // 종료 처리
    rclcpp::shutdown();
    
    return 0;
}