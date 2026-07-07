#ifndef CAN_CSV_LOGGER_HPP
#define CAN_CSV_LOGGER_HPP

#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <mutex>
#include <linux/can.h>
#include <iostream>
#include <filesystem>
#include <cstdlib>

// CommandType은 기존 코드에서 정의됨 (중복 정의 제거)

class CANLogger {
private:
    std::ofstream csv_file;
    std::string log_filename;
    bool logging_enabled;
    std::mutex log_mutex;
    std::chrono::steady_clock::time_point start_time;

public:
    CANLogger() : logging_enabled(false) {
        start_time = std::chrono::steady_clock::now();
    }
    
    ~CANLogger() {
        stop_logging();
    }
    
    bool start_logging() {
        std::lock_guard<std::mutex> lock(log_mutex);
        
        if (logging_enabled) {
            return true; // 이미 로깅 중
        }
        
        // 현재 시간을 파일명에 포함
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;
        
        // 로그 저장 디렉토리: ~/logs/can (없으면 생성). 실패 시 현재 폴더로 폴백
        std::string log_dir;
        if (const char* home = std::getenv("HOME")) {
            log_dir = std::string(home) + "/logs/can";
        } else {
            log_dir = "logs/can";
        }
        std::error_code ec;
        std::filesystem::create_directories(log_dir, ec);
        if (ec) {
            std::cerr << "Failed to create log dir: " << log_dir
                      << " (" << ec.message() << "), using current dir" << std::endl;
            log_dir = ".";
        }

        std::stringstream ss;
        ss << log_dir << "/can_debug_log_"
           << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S")
           << "_" << std::setfill('0') << std::setw(3) << ms.count() << ".csv";
        log_filename = ss.str();
        
        csv_file.open(log_filename);
        if (!csv_file.is_open()) {
            std::cerr << "Failed to open CSV log file: " << log_filename << std::endl;
            return false;
        }
        
        // CSV 헤더 작성
        csv_file << "timestamp_ms,relative_ms,direction,motor_id,can_id_hex,dlc,"
                 << "data0,data1,data2,data3,data4,data5,data6,data7,"
                 << "command_type,position,velocity,acceleration,result_bytes,can_interface\n";
        csv_file.flush();
        
        logging_enabled = true;
        std::cout << "🟢 CSV logging started: " << log_filename << std::endl;
        return true;
    }
    
    void stop_logging() {
        std::lock_guard<std::mutex> lock(log_mutex);
        if (logging_enabled) {
            csv_file.close();
            logging_enabled = false;
            std::cout << "🔴 CSV logging stopped: " << log_filename << std::endl;
        }
    }
    
    void log_sent_command(uint8_t motor_id, const struct can_frame& frame, 
                         int cmd_type, float position = 0.0f, 
                         float velocity = 0.0f, float acceleration = 0.0f, 
                         ssize_t result = 0, const std::string& can_interface = "") {
        std::lock_guard<std::mutex> lock(log_mutex);
        if (!logging_enabled) return;
        
        auto now = std::chrono::steady_clock::now();
        auto absolute_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        auto relative_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - start_time).count();
        
        // timestamp_ms,relative_ms,direction,motor_id,can_id_hex,dlc
        csv_file << absolute_ms << "," << relative_ms << ",SEND," 
                 << static_cast<int>(motor_id) << ",0x" 
                 << std::hex << std::uppercase << frame.can_id << std::dec
                 << "," << static_cast<int>(frame.can_dlc);
        
        // 8바이트 데이터 (없으면 0으로)
        for (int i = 0; i < 8; i++) {
            csv_file << ",";
            if (i < frame.can_dlc) {
                csv_file << "0x" << std::hex << std::uppercase 
                        << std::setfill('0') << std::setw(2) 
                        << static_cast<int>(frame.data[i]) << std::dec;
            } else {
                csv_file << "0x00";
            }
        }
        
        // command_type,position,velocity,acceleration,result_bytes,can_interface
        csv_file << "," << cmd_type
                 << "," << std::fixed << std::setprecision(4) << position 
                 << "," << std::fixed << std::setprecision(2) << velocity 
                 << "," << std::fixed << std::setprecision(2) << acceleration
                 << "," << result << "," << can_interface << "\n";
        csv_file.flush();
    }
    
    void log_received_data(uint8_t motor_id, const struct can_frame& frame, 
                          const std::string& can_interface = "") {
        std::lock_guard<std::mutex> lock(log_mutex);
        if (!logging_enabled) return;
        
        auto now = std::chrono::steady_clock::now();
        auto absolute_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        auto relative_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - start_time).count();
        
        // timestamp_ms,relative_ms,direction,motor_id,can_id_hex,dlc
        csv_file << absolute_ms << "," << relative_ms << ",RECV," 
                 << static_cast<int>(motor_id) << ",0x" 
                 << std::hex << std::uppercase << frame.can_id << std::dec
                 << "," << static_cast<int>(frame.can_dlc);
        
        // 8바이트 데이터
        for (int i = 0; i < 8; i++) {
            csv_file << ",";
            if (i < frame.can_dlc) {
                csv_file << "0x" << std::hex << std::uppercase 
                        << std::setfill('0') << std::setw(2) 
                        << static_cast<int>(frame.data[i]) << std::dec;
            } else {
                csv_file << "0x00";
            }
        }
        
        // 수신 데이터는 명령 정보 없음
        csv_file << ",,,,,," << can_interface << "\n";
        csv_file.flush();
    }
    
    void log_error(const std::string& error_message, uint8_t motor_id = 0) {
        std::lock_guard<std::mutex> lock(log_mutex);
        if (!logging_enabled) return;
        
        auto now = std::chrono::steady_clock::now();
        auto absolute_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        auto relative_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - start_time).count();
        
        csv_file << absolute_ms << "," << relative_ms << ",ERROR," 
                 << static_cast<int>(motor_id) << ",0x0,0,"
                 << "0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,"
                 << ",,,,," << error_message << "\n";
        csv_file.flush();
    }
    
    bool is_logging() {
        std::lock_guard<std::mutex> lock(log_mutex);
        return logging_enabled;
    }
    
    std::string get_filename() {
        std::lock_guard<std::mutex> lock(log_mutex);
        return log_filename;
    }
};

#endif // CAN_CSV_LOGGER_HPP
