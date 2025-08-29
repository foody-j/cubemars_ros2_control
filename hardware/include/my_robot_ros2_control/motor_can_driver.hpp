#ifndef MOTOR_CAN_DRIVER_TEST_HPP
#define MOTOR_CAN_DRIVER_TEST_HPP

#include <string>
#include <iostream>
#include <sstream>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/select.h>
#include <iomanip>
#include <cstdint>
#include <cstring>
#include <map>
#include <vector>
#include <array>
#include <algorithm>
#include <chrono>
#include <thread>
#include <functional>
#include <atomic>
#include <mutex>
#include "motor_data.hpp"
#include "can_csv_logger.hpp"
#include "cubemars_motors.hpp"


struct CanInterface {
    std::string name;
    int socket_fd;
    bool is_connected;
    std::thread read_thread;
    std::atomic<bool> read_running;
    
    CanInterface() : socket_fd(-1), is_connected(false), read_running(false) {}
};

class CanComms
{
public:
    CanComms() : running_(false) {
        motor_manager_.reset();
        motor_to_can_map_.fill(-1);  // -1은 아직 매핑되지 않음을 의미
        
        // 모터 구성 정보 출력
        CubeMars::print_motor_configuration();
        
        // CAN 인터페이스 초기화
        for (int i = 0; i < MAX_CAN_INTERFACES; i++) {
            can_interfaces_[i].name = "can" + std::to_string(i);
        }
    }

    ~CanComms() {
        // 스레드 종료 플래그 설정
        running_ = false;
        
        // 모든 CAN 인터페이스의 읽기 스레드 종료
        for (auto& interface : can_interfaces_) {
            interface.read_running = false;
        }
        
        // 커맨드 스레드 종료 대기
        if (command_thread_.joinable()) {
            command_thread_.join();
        }
        
        // 모든 읽기 스레드 종료 대기
        for (auto& interface : can_interfaces_) {
            if (interface.read_thread.joinable()) {
                interface.read_thread.join();
            }
        }
        
        // 모든 CAN 인터페이스 연결 해제
        for (auto& interface : can_interfaces_) {
            if (interface.is_connected) {
                try {
                    disconnect_interface(interface);
                } catch (const std::exception& e) {
                    std::cerr << "Error during disconnect in destructor: " 
                            << e.what() << std::endl;
                }
            }
        }
    }
    
    // 복사 생성자와 대입 연산자 삭제
    CanComms(const CanComms&) = delete;
    CanComms& operator=(const CanComms&) = delete;

    void connect() {
        static constexpr int32_t BITRATE = 1000000;  // 1Mbps

        try {
            // 가능한 모든 CAN 인터페이스를 찾아 연결 시도
            for (int i = 0; i < MAX_CAN_INTERFACES; i++) {
                try {
                    std::string interface_name = "can" + std::to_string(i);
                    connect_interface(can_interfaces_[i], interface_name, BITRATE);
                    std::cout << "Successfully connected to " << interface_name 
                            << " with bitrate " <<BITRATE << std::endl;
                } catch (const std::exception& e) {
                    std::cerr << "Failed to connect to can" << i << ": " 
                            << e.what() << std::endl;
                    // 이 인터페이스 연결 실패는 무시하고 다음 인터페이스로 진행
                }
            }
            
            // 적어도 하나의 인터페이스가 연결되었는지 확인
            if (std::none_of(can_interfaces_.begin(), can_interfaces_.end(),
                           [](const CanInterface& i) { return i.is_connected; })) {
                throw std::runtime_error("Failed to connect to any CAN interface");
            }
            
            // 명령 스레드 시작
            running_ = true;
            command_thread_ = std::thread(&CanComms::command_loop, this);
        }
        catch(const std::exception& e) {
            // 오류 발생 시 연결된 모든 인터페이스 해제
            disconnect();
            throw;
        }
    }

    void disconnect() {
        running_ = false;
        
        if (command_thread_.joinable()) {
            command_thread_.join();
        }
        
        for (auto& interface : can_interfaces_) {
            if (interface.is_connected) {
                try {
                    disconnect_interface(interface);
                } catch (const std::exception& e) {
                    std::cerr << "Error during disconnect: " <<e.what() <<std::endl;
                }
            }
        }
    }

    bool connected() const {
        // 하나 이상의 인터페이스가 연결되어 있으면 true 반환
        return std::any_of(can_interfaces_.begin(), can_interfaces_.end(),
                         [](const CanInterface& i) { return i.is_connected; });
    }

    // 모터 명령 메서드들 (기존 코드와 유사하지만 모터 ID에 따라 적절한 CAN 인터페이스 선택)

    void write_duty_cycle(uint8_t driver_id, float duty) {
        if (driver_id < 1 || driver_id > MAX_MOTORS) {
            throw std::runtime_error("Invalid motor ID");
        }
        
        // 모터 명령 업데이트
        std::lock_guard<std::mutex> lock(command_mutex_);
        motor_commands_[driver_id - 1].motor_id = driver_id;
        motor_commands_[driver_id - 1].duty = duty; // duty_cycle은 현재로 처리
        motor_commands_[driver_id - 1].active = true;
        motor_commands_[driver_id - 1].command_type = CommandType::DUTY;
        motor_commands_[driver_id - 1].last_sent = std::chrono::steady_clock::now();
    }

    void write_current(uint8_t driver_id, float current_A) {
        if (driver_id < 1 || driver_id > MAX_MOTORS) {
            throw std::runtime_error("Invalid motor ID");
        }
        
        // 모터 명령 업데이트
        std::lock_guard<std::mutex> lock(command_mutex_);
        motor_commands_[driver_id - 1].motor_id = driver_id;
        motor_commands_[driver_id - 1].current = current_A;
        motor_commands_[driver_id - 1].active = true;
        motor_commands_[driver_id - 1].command_type = CommandType::CURRENT; // 현재는 속도 모드로 처리
        motor_commands_[driver_id - 1].last_sent = std::chrono::steady_clock::now();
    }
    void write_velocity(uint8_t driver_id, float rpm) {
        if (driver_id <1 || driver_id > MAX_MOTORS) {
            throw std::runtime_error("Invalid motor ID");
        }
        
        // 모터 명령 업데이트
        std::lock_guard<std::mutex> lock(command_mutex_);
        motor_commands_[driver_id - 1].motor_id = driver_id;
        motor_commands_[driver_id - 1].value = rpm;
        motor_commands_[driver_id - 1].active = true;
        motor_commands_[driver_id - 1].command_type = CommandType::VELOCITY;
        motor_commands_[driver_id - 1].last_sent = std::chrono::steady_clock::now();
    }

    void write_set_origin(uint8_t driver_id, bool is_permanent = false) {
        if (driver_id < 1 || driver_id > MAX_MOTORS) {
            throw std::runtime_error("Invalid motor ID");
        }
        
        // 모터 명령 업데이트
        std::lock_guard<std::mutex> lock(command_mutex_);
        motor_commands_[driver_id - 1].motor_id = driver_id;
        motor_commands_[driver_id - 1].is_permanent = is_permanent;
        motor_commands_[driver_id - 1].active = true;
        motor_commands_[driver_id - 1].command_type = CommandType::SET_ORIGIN;
        motor_commands_[driver_id - 1].last_sent = std::chrono::steady_clock::now();
    }

    void write_position_velocity(uint8_t driver_id, float position, float rpm, float acceleration) {
        if (driver_id < 1 || driver_id > MAX_MOTORS) {
            throw std::runtime_error("Invalid motor ID");
        }
        
        // 모터 명령 업데이트
        std::lock_guard<std::mutex> lock(command_mutex_);
        motor_commands_[driver_id - 1].motor_id = driver_id;
        motor_commands_[driver_id - 1].position = position;
        motor_commands_[driver_id - 1].velocity = rpm;
        motor_commands_[driver_id - 1].acceleration = acceleration;
        motor_commands_[driver_id - 1].active = true;
        motor_commands_[driver_id - 1].command_type = CommandType::POSITION_VELOCITY;
        motor_commands_[driver_id - 1].last_sent = std::chrono::steady_clock::now() - std::chrono::milliseconds(20);  // 과거 시간으로 설정
    }

    // 모터 데이터 조회
    MotorData getMotorData(uint8_t motor_id) {
        if (motor_id < 1 || motor_id > MAX_MOTORS) {
            throw std::runtime_error("Invalid motor ID");
        }
        return motor_manager_.getMotorData(motor_id);
    }

    // motor_manager에 대한 getter
    MotorDataManager& getMotorManager() {
        return motor_manager_;
    }

    // 새로운 듀티 사이클 기반 원점 초기화 함수
    bool initialize_motor_origin_duty_cycle(uint8_t driver_id, float duty_cycle = -0.04f,
                                            float speed_threshold = 0.5f, int timeout_seconds = 10) {
        auto TIMEOUT_DURATION = std::chrono::seconds(timeout_seconds);

        if (driver_id < 1 || driver_id > MAX_MOTORS) {
            throw std::runtime_error("Invalid motor ID");
        }

        // 상태를 관리하기 위한 변수
        enum class HomingState { WATING_FOR_MOVEMENT, WAITING_FOR_STOP };
        HomingState state = HomingState::WATING_FOR_MOVEMENT;
        try {
            std::cout << "듀티 사이클 기반 원점 탐색 시작 (모터 " << static_cast<int>(driver_id)
                      << ", 탐색 듀티 사이클: " << duty_cycle << ")\n";
            
            // 초기 정지
            write_duty_cycle(driver_id, 0.0f);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));

            // 원점 탐색 시작
            write_duty_cycle(driver_id, duty_cycle);

            // 원점 감지 대기
            auto start_time = std::chrono::steady_clock::now();

            while (std::chrono::steady_clock::now() - start_time < TIMEOUT_DURATION) {
                // 개선점: 직접 CAN을 읽지 않고 백그라운드 스레드가 갱신해주는 최신 데이터 사용
                MotorData data = getMotorData(driver_id);
                
                // [1단계] 모터가 움직이기 시작했는지 확인
                if (state == HomingState::WATING_FOR_MOVEMENT) {
                    if (std::abs(data.speed) > speed_threshold) {
                        std::cout << "모터 움직임 감지됨. 이제 정지를 대기합니다... (현재 속도: " 
                        << data.speed << " RPM)\n";
                        state = HomingState::WAITING_FOR_STOP; // 상태를 정지 대기로 변경
                    }
                }
                // [2단계] 모터가 정지했는지 확인
                else if (state == HomingState::WAITING_FOR_STOP) {
                    if (std::abs(data.speed) < speed_threshold) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 안정성을 위해 짧은 시간 대기
                        data = getMotorData(driver_id); // 다시 한번 최신 데이터 확인
                        if (std::abs(data.speed) < speed_threshold) {
                            std::cout << "원점 감지됨 (속도: " << data.speed << " RPM)\n";
                        write_set_origin(driver_id, false);
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        write_duty_cycle(driver_id, 0.0f);
                        return true;
                        }
                    }
                }    
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            std::cout << "원점 초기화 시간 초과\n";
            write_duty_cycle(driver_id, 0.0f); // 안전을 위해 모터 정지
            return false;

        } catch (const std::exception& e) {
            std::cerr << "원점 초기화 실패: " << e.what() << "\n";
            write_duty_cycle(driver_id, 0.0f); // 안전을 위해 모터 정지
            return false;
        }
    }
    
    
    // 원점 초기화 함수 - 임계 토크 파라미터화
    bool initialize_motor_origin(uint8_t driver_id, float current_threshold = 0.4f,
                                 float search_speed = -2.0f, int timeout_seconds = 5) {
        static auto TIMEOUT_DURATION = std::chrono::seconds(timeout_seconds);

        if (driver_id < 1 || driver_id > MAX_MOTORS) {
            throw std::runtime_error("Invalid motor ID");
        }
        // 모터가 매핑된 CAN 인터페이스 확인
        int can_idx = get_can_index_for_motor(driver_id);
        if (can_idx < 0) {
            throw std::runtime_error("Motor not mapped to any CAN interface");
        }

        try {
            std::cout << "원점 탐색 시작 (모터 " << static_cast<int>(driver_id)
                      << ", 임계 전류: " << current_threshold << "A(\n";
            auto start_time = std::chrono::steady_clock::now();
            struct can_frame frame;

            while (std::chrono::steady_clock::now() - start_time <TIMEOUT_DURATION) {
                if (readCanFrame(can_interfaces_[can_idx], frame)) {
                    uint8_t resp_id = frame.can_id & 0xFF;
                    if (resp_id == driver_id) {
                        // 위치 값 확인 
                        int16_t position_raw = (frame.data[0] << 8) | frame.data[1];
                        float position = position_raw * 0.1f;
                        // 전류 값 확인VELOCITY
                        int16_t current_raw = (frame.data[4] << 8) | frame.data[5];
                        float current = current_raw * 0.01f;

                        std::cout << "Position: " << position <<", Current: " <<current << "A\n";

                        if (current > current_threshold) {
                            std::cout << "원점 감지됨: " << current << "A\n";
                            write_set_origin(driver_id, false);
                            write_velocity(driver_id, 0.0f);
                            std::this_thread::sleep_for(std::chrono::milliseconds(100));
                            return true;
                        }
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            std::cout <<"원점 초기화 시간 초과\n";
            write_velocity(driver_id, 0.0f);  // 안전을 위해 모터 정지
            return false;

        } catch (const std::exception& e) {
            std::cerr << "원점 초기화 실패: " << e.what() << "\n";
            write_velocity(driver_id, 0.0f);  // 안전을 위해 모터 정지
            return false;
        }
    }

private:
    // 최대 CAN 인터페이스 수
    static const int MAX_CAN_INTERFACES = 3;
    // 최대 모터 수
    static const int MAX_MOTORS = 6;
    
    // CAN 인터페이스 배열
    std::array<CanInterface, MAX_CAN_INTERFACES> can_interfaces_;
    // 모터 ID → CAN 인터페이스 매핑 (1-based index를 0-based array index로 변환)
    std::array<int, MAX_MOTORS> motor_to_can_map_;
    
    MotorDataManager motor_manager_;
    std::thread command_thread_;
    std::atomic<bool> running_{false};
    std::mutex command_mutex_;
    CANLogger can_logger;  // CSV 로거 인스턴스


    // 명령 타입 열거형
    enum class CommandType {
        DUTY,
        CURRENT,
        VELOCITY,
        POSITION_VELOCITY,
        SET_ORIGIN
    };

    // 모터 명령 구조체
    struct MotorCommand {
        uint8_t motor_id;
        float duty;  // duty_cycle mode에서 사용
        float current; // current mode에서 사용
        float value;  // velocity mode에서 사용
        float position;  // position-velocity mode에서 사용
        float velocity;  // position-velocity mode에서 사용
        float acceleration;  // position-velocity mode에서 사용
        bool is_permanent;  // SET_ORIGIN 모드에서 사용

        std::chrono::steady_clock::time_point last_sent;
        bool active;
        CommandType command_type;
        bool try_all_interfaces; // 모든 인터페이스에 명령 시도
        
        MotorCommand() : motor_id(0), value(0), position(0), velocity(0), 
                        acceleration(0), active(false), current(0),
                        command_type(CommandType::VELOCITY),
                        try_all_interfaces(true) {} // 기본적으로 true로 설정 {}
    };

    std::array<MotorCommand, MAX_MOTORS> motor_commands_;

    // CAN 인터페이스 연결 (단일 인터페이스)
    void connect_interface(CanInterface& interface, const std::string& can_name, int32_t bitrate) {
        try {
            // 1. CAN 인터페이스를 내린다
            std::stringstream ss;
            ss << "ip link set " << can_name <<" down";
            int result = std::system(ss.str().c_str());
            if (result <0) {
                throw std::runtime_error("Failed to set CAN interface down");
            }

            // 2. bitrate 설정
            ss.str("");
            ss << "ip link set " << can_name <<" type can bitrate " << bitrate;
            result = std::system(ss.str().c_str());
            if (result < 0) {
                throw std::runtime_error("Failed to set CAN bitrate");
            }

            // 3. CAN 인터페이스를 올린다이렇게 하면 모터가 아직 매핑되지 않았더라도 명령을 받을 수 있고, 응답하면 자동으로 매핑이 설정되어 이후에는 해당 인터페이스로만 통신하게 됩니다.
            ss.str("");
            ss << "ip link set " << can_name <<" up";
            result = std::system(ss.str().c_str());
            if (result <0) {
                throw std::runtime_error("Failed to set CAN interface up");
            }

            // 4. 소켓 생성
            interface.socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
            if (interface.socket_fd < 0) {
                throw std::runtime_error("Failed to create CAN socket");
            }

            // 5. 인터페이스 이름으로 인덱스 찾기
            struct ifreq ifr;
            std::strcpy(ifr.ifr_name, can_name.c_str());
            if (ioctl(interface.socket_fd, SIOCGIFINDEX, &ifr) < 0) {
                close(interface.socket_fd);
                throw std::runtime_error("Failed to get interface index");
            }

            // 6. 소켓 바인딩
            struct sockaddr_can addr;
            addr.can_family = AF_CAN;
            addr.can_ifindex = ifr.ifr_ifindex;

            if (bind(interface.socket_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
                close(interface.socket_fd);
                throw std::runtime_error("Failed to bind CAN socket");
            }

            // 연결 상태 업데이트
            interface.is_connected = true;
            interface.name = can_name;
            
            // 읽기 스레드 시작
            interface.read_running = true;
            interface.read_thread = std::thread(&CanComms::read_loop, this, std::ref(interface));
        }
        catch(const std::exception& e) {
            if (interface.socket_fd >= 0) {
                close(interface.socket_fd);
                interface.socket_fd = -1;
            }
            interface.is_connected = false;
            throw;
        }
    }

    // CAN 인터페이스 연결 해제 (단일 인터페이스)
    void disconnect_interface(CanInterface& interface) {
        if (interface.is_connected) {
            // 읽기 스레드 종료
            interface.read_running = false;
            if (interface.read_thread.joinable()) {
                interface.read_thread.join();
            }
            
            // 소켓 닫기
            if (interface.socket_fd >= 0) {
                close(interface.socket_fd);
                interface.socket_fd = -1;
            }
            
            // 인터페이스 다운
            std::stringstream ss;
            ss << "ip link set " <<interface.name << " down";
            std::system(ss.str().c_str());
            
            interface.is_connected = false;
        }
    }

    // 특정 CAN 인터페이스에서 CAN 프레임 읽기
    bool readCanFrame(CanInterface& interface, struct can_frame& frame) {
        if (!interface.is_connected || interface.socket_fd < 0) {
            return false;
        }

        fd_set rdfs;
        struct timeval tv{0, 10000};  // 10ms 타임아웃

        FD_ZERO(&rdfs);
        FD_SET(interface.socket_fd, &rdfs);

        if (select(interface.socket_fd + 1, &rdfs, nullptr, nullptr, &tv) <= 0) {
            return false;
        }

        if (read(interface.socket_fd, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            return false;
        }

        return true;
    }

    // 모터 ID에 해당하는 CAN 인터페이스 인덱스 가져오기
    int get_can_index_for_motor(uint8_t motor_id) {
        if (motor_id <1 || motor_id > MAX_MOTORS) {
            return -1;
        }
        
        // 모터 ID는 1부터 시작하므로 배열 인덱스로 변환
        int idx = motor_id - 1;
        
        // 이미 매핑된 정보가 있으면 사용
        if (motor_to_can_map_[idx] >= 0) {
            return motor_to_can_map_[idx];
        }
        
        /*
        // 모터 ID에 따라 CAN 인터페이스 결정 (요구사항에 맞춤)
        if (motor_id <= 2) {
            return 0;  // CAN0: 모터 1,2
        } else if (motor_id <= 4) {
            return 1;  // CAN1: 모터 3,4
        } else {
            return 2;  // CAN2: 모터 5,6
        }*/
       return -1;  // 아직 매핑되지 않음
    }

    // CAN 읽기 루프 (각 인터페이스별)
    void read_loop(CanInterface& interface) {
        while (interface.read_running) {
            struct can_frame frame;
            if (readCanFrame(interface, frame)) {
                uint8_t resp_id = frame.can_id & 0xFF;
                // 🟢 수신 데이터 로깅 추가
                can_logger.log_received_data(resp_id, frame, interface.name);
                if (resp_id >= 1 && resp_id <= MAX_MOTORS) {
                    // 모터 응답을 처음 발견한 경우 CAN 인터페이스 매핑 업데이트
                    int motor_idx = resp_id - 1;
                    int can_idx = &interface - &can_interfaces_[0];  // 포인터 계산으로 인덱스 구하기
                    
                    if (motor_to_can_map_[motor_idx] == -1) {
                        motor_to_can_map_[motor_idx] = can_idx;
                        std::cout << "Motor " <<static_cast<int>(resp_id) 
                                <<" mapped to " << interface.name <<std::endl;
                    }
                    
                    // 모터 데이터 업데이트
                    MotorData data;
                    data.motor_id = resp_id;
                    
                    // 위치 데이터 추출 (data[0-1])
                    int16_t position_raw = (frame.data[0] << 8) | frame.data[1];
                    data.position = CubeMars::Protocol::protocol_to_position(position_raw);

                    // ✅ 속도 데이터 추출 및 변환 (data[2-3])
                    int16_t speed_raw = (frame.data[2] << 8) | frame.data[3];
                    // 기존: data.speed = speed_raw * 10.0f;  // ERPM까지만!
                    // 수정: 프로토콜 → ERPM → 출력 RPM
                    data.speed = CUBEMARS_CONVERT_RECEIVED_VELOCITY(speed_raw, resp_id);

                    // 전류 데이터 추출 (data[4-5])
                    int16_t current_raw = (frame.data[4] << 8) | frame.data[5];
                    data.current = CubeMars::Protocol::protocol_to_current(current_raw);

                    // 온도와 에러 추출 (data[6-7])
                    data.temperature = frame.data[6];
                    data.error = frame.data[7];

                    // 모터 매니저 업데이트
                    motor_manager_.updateMotorData(resp_id, data);
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    void command_loop() {
        static constexpr auto UPDATE_INTERVAL = std::chrono::milliseconds(10);
        
        // 🟢 로깅 시작
        can_logger.start_logging();
        
        while (running_) {
            auto current_time = std::chrono::steady_clock::now();
            
            // 각 모터의 활성 명령 처리
            for (size_t i = 0; i < motor_commands_.size(); i++) {
                MotorCommand cmd_copy;
                {
                    std::lock_guard<std::mutex> lock(command_mutex_);
                    cmd_copy = motor_commands_[i];  // 복사해서 빠르게 락 해제
                }
                
                if (cmd_copy.active) {
                    // std::cout << "*** COMMAND_LOOP: Motor " << static_cast<int>(cmd_copy.motor_id) 
                    //          << " active, type=" << static_cast<int>(cmd_copy.command_type) 
                    //          << ", pos=" << cmd_copy.position << std::endl;
                    
                    // 마지막 전송 후 UPDATE_INTERVAL이 지났는지 확인
                    if (current_time - cmd_copy.last_sent >= UPDATE_INTERVAL) {
                        // std::cout << "*** COMMAND_LOOP: Sending CAN frame for motor " 
                        //          << static_cast<int>(cmd_copy.motor_id) << std::endl;
                        
                        // 해당 모터가 매핑된 CAN 인터페이스 찾기
                        int can_idx = get_can_index_for_motor(cmd_copy.motor_id);
                        
                        // 명령 타입에 따라 CAN 프레임 생성
                        struct can_frame frame{};
                        
                        if (cmd_copy.command_type == CommandType::VELOCITY) {
                            uint32_t control_mode = 3;  // Velocity Mode
                            uint32_t id = (control_mode << 8) | cmd_copy.motor_id;

                            // ✅ 모터 ID로 자동 변환 (모델 선택 불필요!)
                            int32_t speed = CUBEMARS_CONVERT_VELOCITY(cmd_copy.value, cmd_copy.motor_id);
                            
                            // int32_t speed = static_cast<int32_t>(cmd_copy.value);
                            
                            frame.can_id = id | CAN_EFF_FLAG;
                            frame.can_dlc = 4;
                            frame.data[0] = (speed >> 24) & 0xFF;
                            frame.data[1] = (speed >> 16) & 0xFF;
                            frame.data[2] = (speed >> 8) & 0xFF;
                            frame.data[3] = speed & 0xFF;
                            
                        } else if (cmd_copy.command_type == CommandType::POSITION_VELOCITY) {
                            uint32_t control_mode = 6;  // Position-Velocity Mode
                            uint32_t id = (control_mode << 8) | cmd_copy.motor_id;
                            
                            int32_t pos = CubeMars::Protocol::position_to_protocol(cmd_copy.position);
            
                            // ✅ 모터 ID로 자동 변환 (각 모터별로 다른 모델 자동 적용!)
                            int16_t speed = CUBEMARS_CONVERT_POSITION_VELOCITY(cmd_copy.velocity, cmd_copy.motor_id);
                            int16_t acc = CUBEMARS_CONVERT_ACCELERATION(cmd_copy.acceleration, cmd_copy.motor_id);
                            // int32_t pos = static_cast<int32_t>(cmd_copy.position * 10000.0f);
                            // int16_t speed = static_cast<int16_t>(cmd_copy.velocity / 10.0f);
                            // int16_t acc = static_cast<int16_t>(cmd_copy.acceleration / 10.0f);
                            
                            frame.can_id = id | CAN_EFF_FLAG;
                            frame.can_dlc = 8;
                            frame.data[0] = (pos >> 24) & 0xFF;
                            frame.data[1] = (pos >> 16) & 0xFF;
                            frame.data[2] = (pos >> 8) & 0xFF;
                            frame.data[3] = pos & 0xFF;
                            frame.data[4] = (speed >> 8) & 0xFF;
                            frame.data[5] = speed & 0xFF;
                            frame.data[6] = (acc >> 8) & 0xFF;
                            frame.data[7] = acc & 0xFF;
                            
                        } else if (cmd_copy.command_type == CommandType::DUTY) {
                            uint32_t control_mode = 0;  // Duty Cycle Mode
                            uint32_t id = (control_mode << 8) | cmd_copy.motor_id;

                            int32_t duty_cycle = static_cast<int32_t>(cmd_copy.duty * 100000.0f); 

                            frame.can_id = id | CAN_EFF_FLAG;
                            frame.can_dlc = 4;
                            frame.data[0] = (duty_cycle >> 24) & 0xFF;
                            frame.data[1] = (duty_cycle >> 16) & 0xFF;
                            frame.data[2] = (duty_cycle >> 8) & 0xFF;
                            frame.data[3] = duty_cycle & 0xFF;
                        } else if (cmd_copy.command_type == CommandType::SET_ORIGIN) {
                            uint32_t control_mode = 5;  // Set Origin Mode
                            uint32_t id = (control_mode << 8) | cmd_copy.motor_id;
                            
                            frame.can_id = id | CAN_EFF_FLAG;
                            frame.can_dlc = 1;
                            frame.data[0] = cmd_copy.is_permanent ? 1 : 0;
                        } else if (cmd_copy.command_type == CommandType::CURRENT) {
                            uint32_t control_mode = 1;  // Current Mode
                            uint32_t id = (control_mode << 8) | cmd_copy.motor_id;
                            
                            int32_t current_mA = static_cast<int32_t>(cmd_copy.current * 1000.0f);  // mA 단위로 변환

                            frame.can_id = id | CAN_EFF_FLAG;
                            frame.can_dlc = 4;
                            frame.data[0] = (current_mA >> 24) & 0xFF;
                            frame.data[1] = (current_mA >> 16) & 0xFF;
                            frame.data[2] = (current_mA >> 8) & 0xFF;
                            frame.data[3] = current_mA & 0xFF;
                        } else {
                            std::cerr << "Unknown command type for motor " 
                                    << static_cast<int>(cmd_copy.motor_id) << std::endl;
                            continue; // 알 수 없는 명령 타입은 건너뜀
                        }
                        
                        // 매핑된 인터페이스가 있으면 해당 인터페이스로만 전송
                        if (can_idx >= 0 && can_idx < MAX_CAN_INTERFACES && 
                            can_interfaces_[can_idx].is_connected) {
    
                            // 기존 디버깅 로그는 주석 처리 
                            // std::cout << "*** DEBUG CAN Motor " << static_cast<int>(cmd_copy.motor_id)
                            //         << ": ID=0x" << std::hex << frame.can_id
                            //         << ", DLC=" << std::dec << (int)frame.can_dlc
                            //         << ", Data=[";
                            // for(int k=0; k < frame.can_dlc; ++k) {
                            //     std::cout << "0x" << std::hex << static_cast<int>(frame.data[k]) << " ";
                            // }
                            // std::cout << std::dec << "]" << std::endl;
                            
                            // CAN 메시지 전송
                            ssize_t result = write(can_interfaces_[can_idx].socket_fd, &frame, sizeof(struct can_frame));
                            
                            // 🟢 CSV 로깅 (기존 디버그 출력 대신)
                            can_logger.log_sent_command(cmd_copy.motor_id, frame, static_cast<int>(cmd_copy.command_type),
                                                       cmd_copy.position, cmd_copy.velocity, 
                                                       cmd_copy.acceleration, result, 
                                                       can_interfaces_[can_idx].name);
                            
                            // 필요시 간단한 출력만 남기기
                            if (result != sizeof(struct can_frame)) {
                                 // std::cout << "⚠️  CAN write error: " << result << " bytes" << std::endl;
                            }
                            
                        }
                        // 매핑되지 않았으면 모든 연결된 인터페이스로 전송
                        else if (cmd_copy.try_all_interfaces) {
                            // std::cout << "*** Sending to all interfaces (not mapped yet)" << std::endl;
                            
                            for (int j = 0; j < MAX_CAN_INTERFACES; j++) {
                                if (can_interfaces_[j].is_connected) {
                                    // CAN 메시지 전송
                                    ssize_t result = write(can_interfaces_[j].socket_fd, &frame, sizeof(struct can_frame));
                                    
                                    // 🟢 CSV 로깅 (모든 인터페이스별로)
                                    can_logger.log_sent_command(cmd_copy.motor_id, frame, static_cast<int>(cmd_copy.command_type),
                                                               cmd_copy.position, cmd_copy.velocity, 
                                                               cmd_copy.acceleration, result, 
                                                               can_interfaces_[j].name);
                                    
                                    // 기존 디버그 출력 주석 처리
                                    // std::cout << "*** CAN write to " << can_interfaces_[j].name 
                                    //          << " result: " << result << " bytes" << std::endl;
                                    
                                    // 에러시에만 출력
                                    if (result != sizeof(struct can_frame)) {
                                         // std::cout << "⚠️  CAN write error to " << can_interfaces_[j].name 
                                            //      << ": " << result << " bytes" << std::endl;
                                    }
                                }
                            }
                        }
                        
                        // last_sent 업데이트
                        {
                            std::lock_guard<std::mutex> lock(command_mutex_);
                            motor_commands_[i].last_sent = current_time;
                        }
                    } else {
                        // 대기 시간 출력도 주석 처리 (너무 많은 출력 방지)
                        // auto time_left = std::chrono::duration_cast<std::chrono::milliseconds>
                        //                (UPDATE_INTERVAL - (current_time - cmd_copy.last_sent));
                        // std::cout << "*** COMMAND_LOOP: Waiting " << time_left.count() 
                        //          << "ms for motor " << static_cast<int>(cmd_copy.motor_id) << std::endl;
                    }
                }
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        
        // 🟢 로깅 종료
        can_logger.stop_logging();
    }
};

#endif // MOTOR_CAN_DRIVER_HPP