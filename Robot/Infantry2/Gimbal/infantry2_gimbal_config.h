#ifndef __INFANTRY2_GIMBAL_CONFIG_H__
#define __INFANTRY2_GIMBAL_CONFIG_H__

namespace infantry2_gimbal {

constexpr float PITCH_MOTOR_OFFSET{0.0f};
constexpr float GRAVITY_OFFSET{0.762f};
constexpr float PITCH_MAX_MOTOR_RAD{1.29f}, PITCH_MIN_MOTOR_RAD{0.26f};
constexpr float PITCH_MAX_MOTOR_RADPS{30.0f}, PITCH_MIN_MOTOR_RADPS{-30.0f};
constexpr float PITCH_MAX_MOTOR_TORQUE{7.0f}, PITCH_MIN_MOTOR_TORQUE{-7.0f};

constexpr float YAW_MAX_MOTOR_TORQUE{1.0f}, YAW_MIN_MOTOR_TORQUE{-1.0f};

constexpr float RC_PITCH_COEFFICIENT{0.005f}, RC_YAW_COEFFICIENT{0.002f};

} // namespace infantry2_gimbal

#endif // __INFANTRY2_GIMBAL_CONFIG_H__