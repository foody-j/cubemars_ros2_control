#ifndef TEACHING_DATA_LOGGER_HPP
#define TEACHING_DATA_LOGGER_HPP

#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <mutex>
#include <iostream>
#include <string>

class SimpleTeachingLogger {
private:
    std::ofstream csv_file;
    std::string log_filename;
    bool logging_enabled;
    std::mutex log_mutex;
    std::chrono::steady_clock::time_point start_time;
    int frame_count;

public:
    SimpleTeachingLogger() : logging_enabled(false), frame_count(0) {}
    
    ~SimpleTeachingLogger() {
        stop_teaching();
    }
    
    bool start_teaching(const std::string& template_name = "teaching") {
        std::lock_guard<std::mutex> lock(log_mutex);
        
        if (logging_enabled) {
            std::cout << "⚠️  Teaching already in progress!" << std::endl;
            return false;
        }
        
        // 현재 시간을 파일명에 포함
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;
        
        std::stringstream ss;
        ss << template_name << "_teaching_" 
           << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S")
           << "_" << std::setfill('0') << std::setw(3) << ms.count() << ".csv";
        log_filename = ss.str();
        
        csv_file.open(log_filename);
        if (!csv_file.is_open()) {
            std::cerr << "❌ Failed to open teaching log file: " << log_filename << std::endl;
            return false;
        }
        
        // CSV 헤더 작성 (LSTM 학습용 풍부한 데이터)
        csv_file << "timestamp_ms,timestamp_sec,frame_count,"
                 << "joint1_rad,joint2_rad,joint3_rad,joint4_rad,joint5_rad,joint6_rad,"
                 << "joint1_vel_rpm,joint2_vel_rpm,joint3_vel_rpm,joint4_vel_rpm,joint5_vel_rpm,joint6_vel_rpm,"
                 << "joint1_current_A,joint2_current_A,joint3_current_A,joint4_current_A,joint5_current_A,joint6_current_A,"
                 << "joint1_deg,joint2_deg,joint3_deg,joint4_deg,joint5_deg,joint6_deg\n";
        csv_file.flush();
        
        // 시작 시간 설정
        start_time = std::chrono::steady_clock::now();
        frame_count = 0;
        logging_enabled = true;
        
        std::cout << "🔴 Teaching started: " << log_filename << std::endl;
        return true;
    }
    
    void log_frame(const double joint_positions_rad[6], 
                   const float joint_velocities_rpm[6] = nullptr,
                   const float joint_currents_A[6] = nullptr) {
        if (!logging_enabled) return;
        
        std::lock_guard<std::mutex> lock(log_mutex);
        
        auto now = std::chrono::steady_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - start_time).count();
        auto elapsed_sec = elapsed_ms / 1000.0;
        
        frame_count++;
        
        // timestamp_ms, timestamp_sec, frame_count
        csv_file << elapsed_ms << "," << std::fixed << std::setprecision(3) << elapsed_sec << "," << frame_count;
        
        // joint positions in radians (for LSTM input)
        for (int i = 0; i < 6; i++) {
            csv_file << "," << std::fixed << std::setprecision(6) << joint_positions_rad[i];
        }
        
        // joint velocities in RPM (for LSTM input)
        for (int i = 0; i < 6; i++) {
            if (joint_velocities_rpm) {
                csv_file << "," << std::fixed << std::setprecision(2) << joint_velocities_rpm[i];
            } else {
                csv_file << ",0.0";
            }
        }
        
        // joint currents in Ampere (for force/torque information)
        for (int i = 0; i < 6; i++) {
            if (joint_currents_A) {
                csv_file << "," << std::fixed << std::setprecision(3) << joint_currents_A[i];
            } else {
                csv_file << ",0.0";
            }
        }
        
        // joint positions in degrees (for human reading)
        for (int i = 0; i < 6; i++) {
            csv_file << "," << std::fixed << std::setprecision(2) << (joint_positions_rad[i] * 180.0 / M_PI);
        }
        
        csv_file << "\n";
        csv_file.flush();
        
        // 진행 상황 출력 (매 1초마다)
        if (frame_count % 100 == 0) {  // 100Hz면 1초마다
            std::cout << "📊 Teaching frames: " << frame_count 
                     << " (Duration: " << elapsed_ms / 1000.0 << "s)" << std::endl;
        }
    }
    
    void stop_teaching() {
        std::lock_guard<std::mutex> lock(log_mutex);
        
        if (logging_enabled) {
            auto end_time = std::chrono::steady_clock::now();
            auto total_duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                end_time - start_time).count();
            
            csv_file.close();
            logging_enabled = false;
            
            std::cout << "⏹️  Teaching stopped: " << log_filename << std::endl;
            std::cout << "📈 Summary:" << std::endl;
            std::cout << "   - Total frames: " << frame_count << std::endl;
            std::cout << "   - Duration: " << total_duration_ms / 1000.0 << "s" << std::endl;
            std::cout << "   - Average rate: " << (frame_count * 1000.0 / total_duration_ms) << " Hz" << std::endl;
            
            frame_count = 0;
        }
    }
    
    bool is_teaching() const {
        return logging_enabled;
    }
    
    std::string get_filename() const {
        return log_filename;
    }
    
    int get_frame_count() const {
        return frame_count;
    }
};

#endif // TEACHING_DATA_LOGGER_HPP