#ifndef CUBEMARS_MOTORS_HPP
#define CUBEMARS_MOTORS_HPP

#include <cstdint>

// CubeMars 모터 사양 정의
namespace CubeMars {
    
    // 모터 모델별 사양 구조체
    struct MotorSpecs {
        int pole_pairs;
        int reduction_ratio;
        float max_output_rpm;
        float max_torque_nm;
        const char* name;
    };
    
    // 모터 모델 열거형
    enum class MotorModel {
        AK60_84 = 0,
        AK70_10 = 1,
        AK60_6 = 2
    };
    
    // 모터별 사양 테이블 (매뉴얼에서 확인 필요한 값들은 추정값으로 표시)
    static constexpr MotorSpecs MOTOR_SPECS[] = {
        // pole_pairs, reduction_ratio, max_rpm, max_torque, name
        {21, 64, 23, 48, "AK60-84"},     
        {21, 10, 148, 8.3, "AK70-10"},    
        {14, 6, 420, 3, "AK60-6"}       
    };
    
    // 모터 사양 가져오기
    inline const MotorSpecs& get_motor_specs(MotorModel model) {
        return MOTOR_SPECS[static_cast<int>(model)];
    }
    
    // RPM → ERPM 변환 팩터 계산
    inline float get_rpm_to_erpm_factor(MotorModel model) {
        const auto& specs = get_motor_specs(model);
        return static_cast<float>(specs.reduction_ratio * specs.pole_pairs);
    }
    // ✅ ERPM → 출력축 RPM 변환 (새로 추가!)
    inline float erpm_to_output_rpm(float erpm, MotorModel model) {
        return erpm / get_rpm_to_erpm_factor(model);
    }
    
    // 출력축 RPM → ERPM 변환
    inline float output_rpm_to_erpm(float output_rpm, MotorModel model) {
        return output_rpm * get_rpm_to_erpm_factor(model);
    }
    
    // CubeMars 프로토콜 변환 함수들
    namespace Protocol {
        
        // 위치: 도 → 프로토콜 값
        inline int32_t position_to_protocol(float position_deg) {
            return static_cast<int32_t>(position_deg * 10000.0f);
        }
        
        // 속도: ERPM → 프로토콜 값 (Velocity 모드)
        inline int32_t velocity_to_protocol_int32(float erpm) {
            return static_cast<int32_t>(erpm);
        }
        
        // 속도: ERPM → 프로토콜 값 (Position-Velocity 모드)
        inline int16_t velocity_to_protocol_int16(float erpm) {
            int16_t result = static_cast<int16_t>(erpm / 10.0f);
            // 범위 제한
            if (result > 32767) result = 32767;
            if (result < -32767) result = -32767;
            return result;
        }
        
        // ✅ 올바른 파라미터명
        inline int16_t acceleration_to_protocol(float erpm_per_s2) {  // erpm_per_s2는 가속도!
            int16_t result = static_cast<int16_t>(erpm_per_s2 / 10.0f);
            // 범위 제한 (가속도는 양수만)
            if (result > 32767) result = 32767;
            if (result < 0) result = 0;
            return result;
        }

        // ✅ 프로토콜 값 → 속도 (출력 RPM) - 수신용 (새로 추가!)
        inline float protocol_int16_to_velocity(int16_t protocol_value, MotorModel model) {
            float erpm = protocol_value * 10.0f;  // 프로토콜 → ERPM
            return erpm_to_output_rpm(erpm, model);  // ERPM → 출력 RPM
        }
        // ✅ 프로토콜 값 → 위치 (도) - 수신용
        inline float protocol_to_position(int16_t protocol_value) {
            return protocol_value * 0.1f;
        }
        // ✅ 프로토콜 값 → 전류 (A) - 수신용
        inline float protocol_to_current(int16_t protocol_value) {
            return protocol_value * 0.01f;
        }    

    }
    
    // 모터 ID별 모델 매핑 함수
    inline MotorModel get_motor_model_by_id(uint8_t motor_id) {
        switch (motor_id) {
            case 1:
            case 2:
                return MotorModel::AK60_84;  // 1,2번: AK60-84 (80모터)
            case 3:
            case 4:
                return MotorModel::AK70_10;  // 3,4번: AK70-10 (70모터)
            case 5:
            case 6:
                return MotorModel::AK60_6;   // 5,6번: AK60-6 (60모터)
            default:
                return MotorModel::AK60_84;  // 기본값
        }
    }
    
    // 디버그 정보 출력
    inline void print_motor_info(MotorModel model) {
        const auto& specs = get_motor_specs(model);
        float factor = get_rpm_to_erpm_factor(model);
        
        printf("=== %s 모터 정보 ===\n", specs.name);
        printf("극쌍수: %d\n", specs.pole_pairs);
        printf("감속비: %d:1\n", specs.reduction_ratio);
        printf("최대 출력: %.0f RPM, %.1f Nm\n", specs.max_output_rpm, specs.max_torque_nm);
        printf("변환 팩터: %.0f (출력 RPM × %.0f = ERPM)\n", factor, factor);
        printf("예시: 출력 10 RPM → %.0f ERPM → 프로토콜 %d\n", 
               10.0f * factor, Protocol::velocity_to_protocol_int16(10.0f * factor));
        printf("========================\n");
    }
    
    // 전체 모터 구성 정보 출력
    inline void print_motor_configuration() {
        printf("=== 모터 구성 정보 ===\n");
        printf("모터 1,2번: %s (80모터)\n", get_motor_specs(MotorModel::AK60_84).name);
        printf("모터 3,4번: %s (70모터)\n", get_motor_specs(MotorModel::AK70_10).name);
        printf("모터 5,6번: %s (60모터)\n", get_motor_specs(MotorModel::AK60_6).name);
        printf("===================\n");
    }
}

// 사용 편의를 위한 매크로 (모터 ID 자동 감지)
#define CUBEMARS_CONVERT_VELOCITY(output_rpm, motor_id) \
    CubeMars::Protocol::velocity_to_protocol_int32(CubeMars::output_rpm_to_erpm(output_rpm, CubeMars::get_motor_model_by_id(motor_id)))

#define CUBEMARS_CONVERT_POSITION_VELOCITY(output_rpm, motor_id) \
    CubeMars::Protocol::velocity_to_protocol_int16(CubeMars::output_rpm_to_erpm(output_rpm, CubeMars::get_motor_model_by_id(motor_id)))

#define CUBEMARS_CONVERT_ACCELERATION(output_rpm_per_s2, motor_id) \
    CubeMars::Protocol::acceleration_to_protocol(CubeMars::output_rpm_to_erpm(output_rpm_per_s2, CubeMars::get_motor_model_by_id(motor_id)))

#define CUBEMARS_CONVERT_RECEIVED_VELOCITY(protocol_value, motor_id) \
    CubeMars::Protocol::protocol_int16_to_velocity(protocol_value, CubeMars::get_motor_model_by_id(motor_id))    
#endif // CUBEMARS_MOTORS_HPP