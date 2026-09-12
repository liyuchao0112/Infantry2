#ifndef __INFANTRY2_CONFIG_H__
#define __INFANTRY2_CONFIG_H__

#include "pyro_core_def.h"

#define GIMBAL_EN 1
#define BOOSTER_EN 1
#define CHASSIS_EN 1

#define REFEREE_EN 1

// ---- 板级总线选择 ----
// 同一条物理板间线，两板本地编号不同：云台 = can1，底盘 = can3
// ⚠ 故意用宏而非 constexpr：本文件被模块头包含，
//   若用 constexpr bsp_can::which_can 就需在此 include pyro_bsp_can.h，
//   会把 CAN BSP 依赖污染到模块层。宏不产生解析依赖。
#if BOARD == GIMBAL_BOARD
#define BOARD_COMM_CAN pyro::bsp_can::can1
#elif BOARD == CHASSIS_BOARD
#define BOARD_COMM_CAN pyro::bsp_can::can3
#endif

// ---- 失联超时保护 ----
// *_ENABLE = 1 启用；= 0 关闭（失联后保持最后一帧值）
// *_MS     失联判定阈值（毫秒）

// 0x100 云台→底盘 遥控指令：失联后归零并切 PASSIVE（停车）
#define BOARD_COMM_TIMEOUT_CHASSIS_CMD_ENABLE 1
#define BOARD_COMM_TIMEOUT_CHASSIS_CMD_MS     50

// 0x101 底盘→云台 裁判数据：失联后热量不可信
// 裁判系统本身约 10Hz，阈值需比控制链路宽松
#define BOARD_COMM_TIMEOUT_REFEREE_ENABLE     1
#define BOARD_COMM_TIMEOUT_REFEREE_MS         500

#if BOARD == GIMBAL_BOARD

namespace infantry2_gimbal {

    constexpr float PITCH_MOTOR_OFFSET{0.0f};
    constexpr float PITCH_MAX_MOTOR_RAD{0.21f}, PITCH_MIN_MOTOR_RAD{-0.07f};
    constexpr float PITCH_IMU_OFFSET{0.0f};
    constexpr float PITCH_MAX_IMU_RAD{0.31f}, PITCH_MIN_IMU_RAD{0.04f};

    constexpr float PITCH_MAX_RADPS{30.0f}, PITCH_MIN_RADPS{-30.0f};
    constexpr float PITCH_MAX_MOTOR_TORQUE{7.0f}, PITCH_MIN_MOTOR_TORQUE{-7.0f};

    constexpr float GRAVITY_OFFSET{0.762f};

    constexpr float YAW_MOTOR_OFFSET{0.0f};
    constexpr float YAW_MAX_MOTOR_TORQUE{1.0f}, YAW_MIN_MOTOR_TORQUE{-1.0f};

    constexpr float RC_PITCH_COEFFICIENT{0.005f}, RC_YAW_COEFFICIENT{0.005f};
    constexpr float AUTOAIM_YAW_COEFFICIENT{0.002f}, AUTOAIM_PITCH_COEFFICIENT{0.002f};
    
} // namespace infantry2_gimbal

namespace infantry2_booster {

    constexpr float FRIC_RADIUS{0.03f};
    constexpr float FRIC_RADPS_TOLERANCE{100.0f}, FRIC_RADPS_DEADZONE{30.0f};
    constexpr float FRIC_SHOOT_TORQUE_THRESHOLD{2.5f};
    
    constexpr float TRIGGER_REDUCTION_RATIO{36.0f};
    constexpr float TRIGGER_CONTINUOUS_RADPS{10.0f};
    constexpr float TRIGGER_RAD_TOLERANCE{0.01f}, TRIGGER_RAD_DEADZONE{0.05f};
    
    constexpr float CALI_REVERSE_RADPS{3.0f};
    constexpr float CALI_FORWARD_RAD{0.705045104f-0.0277180634f};
    // constexpr float CALI_FORWARD_RAD{0.522733748f+0.33131066f};
    
    constexpr uint32_t BLOCK_TIME_THRESHOLD{500};
    constexpr float BLOCK_SPD_ERROR_RATE_THRESHOLD{0.5f};
    constexpr float BLOCK_RAD_THRESHOLD{pyro::PI / 16.0f}, BLOCK_SPD_THRESHOLD{0.3f};
    
    constexpr float TARGET_BULLET_SPEED{18.5f};
    
    constexpr float SINGLE_BULLET_RAD{pyro::PI / 4.0f};
    
} // namespace infantry2_booster

#elif BOARD == CHASSIS_BOARD

namespace infantry2_chassis {
    constexpr float YAW_MOTOR_OFFSET{-1.75027227f};
    constexpr float RUDDER_MOTOR_OFFSET[4]{1.6007087f, 0.3014272f, 0.260048148f, 3.01427245f};

    constexpr float RUDDER_MAX_RAD_DELTA{6.0f};

    constexpr float YAW_DEADZONE{0.01f};
    
    constexpr float WHEELBASE{1.41421356237f}, TRACK_WIDTH{1.41421356237f};

    constexpr float WHEEL_RADIUS{0.076f};

    constexpr float SPIN_SPEED{30.0f};

    // ===== 板间遥控器指令 -> 速度控制参数 (底盘侧) =====
    constexpr float MAX_VX{40.0f};              // m/s    前后限速
    constexpr float MAX_VY{40.0f};              // m/s    左右限速
    constexpr float MAX_WZ{16.0f};              // rad/s  旋转限速
    constexpr float MAX_ACCEL{20.0f};           // 加速度限幅 (平移 m/s², 旋转 rad/s²)
    constexpr uint32_t LOST_TIMEOUT_MS{50};    // 掉线停车阈值 (ms)

} //namespace infantry2_chassis

#endif

#endif // __INFANTRY2_CONFIG_H__