#ifndef __INFANTRY2_CONFIG_H__
#define __INFANTRY2_CONFIG_H__

#if BOARD == GIMBAL_BOARD

namespace infantry2_gimbal {

    constexpr float PITCH_MOTOR_OFFSET{0.0f};
    constexpr float GRAVITY_OFFSET{0.762f};
    constexpr float PITCH_MAX_MOTOR_RAD{1.29f}, PITCH_MIN_MOTOR_RAD{0.26f};
    constexpr float PITCH_MAX_MOTOR_RADPS{30.0f}, PITCH_MIN_MOTOR_RADPS{-30.0f};
    constexpr float PITCH_MAX_MOTOR_TORQUE{7.0f}, PITCH_MIN_MOTOR_TORQUE{-7.0f};
    
    constexpr float YAW_MOTOR_OFFSET{0.0f};
    constexpr float YAW_MAX_MOTOR_TORQUE{1.0f}, YAW_MIN_MOTOR_TORQUE{-1.0f};
    
    constexpr float RC_PITCH_COEFFICIENT{0.005f}, RC_YAW_COEFFICIENT{0.002f};
    
} // namespace infantry2_gimbal

namespace infantry2_booster {

    constexpr float FRIC_RADIUS{0.03f};
    constexpr float FRIC_RADPS_TOLERANCE{100.0f}, FRIC_RADPS_DEADZONE{30.0f};
    constexpr float FRIC_SHOOT_TORQUE_THRESHOLD{2.5f};
    
    constexpr float TRIGGER_REDUCTION_RATIO{36.0f};
    constexpr float TRIGGER_CONTINUOUS_RADPS{10.0f};
    constexpr float TRIGGER_RAD_TOLERANCE{0.01f}, TRIGGER_RAD_DEADZONE{0.05f};
    
    constexpr float CALI_REVERSE_RADPS{3.0f};
    constexpr float CALI_FORWARD_RAD{0.0f};
    // constexpr float CALI_FORWARD_RAD{0.522733748f+0.33131066f};
    
    constexpr uint32_t BLOCK_TIME_THRESHOLD{500};
    constexpr float BLOCK_SPD_ERROR_RATE_THRESHOLD{0.5f};
    constexpr float BLOCK_RAD_THRESHOLD{pyro::PI / 16.0f}, BLOCK_SPD_THRESHOLD{0.3f};
    
    constexpr float TARGET_BULLET_SPEED{18.5f};
    
    constexpr float SINGLE_BULLET_RAD{pyro::PI / 4.0f};
    
} // namespace infantry2_booster

#elif BOARD == CHASSIS_BOARD

#endif

#endif // __INFANTRY2_CONFIG_H__