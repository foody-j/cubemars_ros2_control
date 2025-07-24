#ifndef MOTOR_DATA_HPP
#define MOTOR_DATA_HPP

#include <array>
#include <cstdint>

struct MotorData {
    uint8_t motor_id{0};     // 모터 ID (1~6)
    float position{0.0f};    // -3200° ~ +3200°
    float speed{0.0f};       // -32000 ~ +32000 rpm
    float current{0.0f};     // -60A ~ +60A
    int8_t temperature{0};   // -20°C ~ 127°C
    uint8_t error{0};        // 0~7 error codes
};

class MotorDataManager {
public:
    static constexpr size_t MAX_MOTORS = 6;
    
    MotorData& getMotorData(uint8_t motor_id) {
        if (motor_id < 1 || motor_id > MAX_MOTORS) {
            throw std::runtime_error("Invalid motor ID");
        }
        motor_data_[motor_id - 1].motor_id = motor_id; // ID 자동 설정
        return motor_data_[motor_id - 1];
    }

    void updateMotorData(uint8_t motor_id, const MotorData& data) {
        if (motor_id < 1 || motor_id > MAX_MOTORS) {
            throw std::runtime_error("Invalid motor ID");
        }
        motor_data_[motor_id - 1] = data;
        motor_data_[motor_id - 1].motor_id = motor_id; // ID 보장
    }

    void reset() {
        for (uint8_t i = 0; i < MAX_MOTORS; ++i) {
            motor_data_[i] = MotorData{};
            motor_data_[i].motor_id = i + 1; // 리셋 시에도 ID 유지
        }
    }

    // 편의 함수 추가
    float getMotorPosition(uint8_t motor_id) {
        return getMotorData(motor_id).position;
    }

    float getMotorSpeed(uint8_t motor_id) {
        return getMotorData(motor_id).speed;
    }

    float getMotorCurrent(uint8_t motor_id) {
        return getMotorData(motor_id).current;
    }

    int8_t getMotorTemperature(uint8_t motor_id) {
        return getMotorData(motor_id).temperature;
    }

    uint8_t getMotorError(uint8_t motor_id) {
        return getMotorData(motor_id).error;
    }
private:
    std::array<MotorData, MAX_MOTORS> motor_data_;
};

#endif // MOTOR_DATA_HPP