#ifndef __PYRO_BOARD_MSG_H__
#define __PYRO_BOARD_MSG_H__

#include <cstdint>

namespace pyro {

// ============================================================================
// 板间 CAN 通信协议 (云台板 <-> 底盘板)
// ----------------------------------------------------------------------------
// 说明:
//   1. 使用 packed 结构体定义协议, 经 board_comm_t 模板接口收发;
//      收发两端各做一次整块 memcpy, 位打包交给编译器完成。
//   2. 两端必须使用同一编译器 (gcc-arm-none-eabi, 位域 LSB 优先),
//      才能保证位域布局一致。
//   3. 单个消息必须 <= 8 字节 (经典 CAN 单帧上限), 超长请拆分为多个 ID。
//   4. mode/state 取值与底盘侧 cmd_base_t::mode_t /
//      infantry2_chassis_cmd_t::state_t 的枚举顺序对齐, 传输时直接整型互转。
// ============================================================================

// ---------------------------------------------------------------------------
// 板间 CAN ID 分配表
// ---------------------------------------------------------------------------
// CAN 标准帧: ID 数值越小, 仲裁优先级越高。
// !! 本表避开同总线电机反馈/控制帧: 0x1FF, 0x2FF, 0x200~0x20B
//    (云台 can1: GM6020 id_5 -> 0x209, M2006 id_3 -> 0x203)
//    (底盘 can3: GM6020 id_5 -> 0x209)
//    !! 新增 ID 前必须重新核查是否与总线其他设备冲突
// ---------------------------------------------------------------------------
namespace board_comm_id {

// ---- 高优先级实时段 0x080~0x08F (预留, 1kHz 级动态反馈) ----
constexpr uint32_t CHASSIS_POSE = 0x080;  // [预留] 底盘姿态: yaw_rad + yaw_radps
constexpr uint32_t CHASSIS_VEL  = 0x081;  // [预留] 底盘速度: vx + vy + wz (需先补正解)
constexpr uint32_t GIMBAL_POSE  = 0x082;  // [预留] 云台姿态 (若底盘需要)
constexpr uint32_t HI_SEG_BEGIN = 0x080;
constexpr uint32_t HI_SEG_END   = 0x08F;

// ---- 常规段 0x100~0x13F ----
constexpr uint32_t CHASSIS_CMD  = 0x100;  // 已用: 云台 -> 底盘 遥控控制指令
constexpr uint32_t REFEREE_DATA = 0x101;  // 已用: 底盘 -> 云台 裁判射击数据

} // namespace board_comm_id

// 段不重叠校验
static_assert(board_comm_id::HI_SEG_END < board_comm_id::CHASSIS_CMD,
              "board comm ID segments must not overlap");

// mode 取值 (与 cmd_base_t::mode_t 对齐: PASSIVE=0, ACTIVE=1)
constexpr uint8_t MODE_PASSIVE = 0;
constexpr uint8_t MODE_ACTIVE  = 1;

// state 取值 (与 infantry2_chassis_cmd_t::state_t 对齐: NORMAL=0, FOLLOW_YAW=1, SPIN=2)
constexpr uint8_t STATE_NORMAL     = 0;
constexpr uint8_t STATE_FOLLOW_YAW = 1;
constexpr uint8_t STATE_SPIN       = 2;

// 归一化定点比例: 摇杆 float(-1..1) <-> int(-1023..1023)
constexpr int16_t CHASSIS_RC_SCALE = 1023;

// ============================================================================
// 0x100 云台 -> 底盘: 遥控器控制指令
// ============================================================================
// 位域协议结构体 (2+2+11+11+11+8+5 = 50 bit = 6.25 B, 单个 CAN 帧可容纳)
// !! 结构布局保持不变, 保证两板可灰度升级
struct infantry2_g2c_cmd_t {
    uint8_t mode   : 2;   // MODE_PASSIVE / MODE_ACTIVE (留1位扩展)
    uint8_t state  : 2;   // STATE_NORMAL / STATE_FOLLOW_YAW / STATE_SPIN
    int16_t vx     : 11;  // 前后归一化 -1023..1023
    int16_t vy     : 11;  // 左右归一化 -1023..1023
    int16_t wz     : 11;  // 旋转归一化 -1023..1023
    uint8_t seq    : 8;   // 心跳帧计数
    uint8_t rsvd   : 5;   // 预留, 以后加字段用

    static constexpr uint32_t ID = board_comm_id::CHASSIS_CMD;
} __attribute__((packed));
static_assert(sizeof(infantry2_g2c_cmd_t) <= 8,
              "infantry2_g2c_cmd_t must fit one CAN frame");

// ============================================================================
// 0x101 底盘 -> 云台: 裁判系统数据
// ============================================================================
// 字段名与 RM 裁判系统协议保持一致, 便于追溯
// (见 PYRo/Component/Referee/protocol.h)
//
// 位宽按实际值域裁剪, 压缩后 43 bit + 21 bit 预留 = 64 bit = 单帧
// 拆为两个 32 bit 字, 保证布局不依赖编译器的跨字打包策略
struct infantry2_c2g_referee_msg_t {
    uint32_t initial_speed_x100           : 16;  // shoot_data.initial_speed x100
                                                 //   0.01 m/s 精度
    uint32_t shooter_17mm_barrel_heat     : 16;  // power_heat.shooter_17mm_barrel_heat
                                                 //   当前枪口热量
    uint32_t shooter_barrel_heat_limit    : 9;   // robot_status.shooter_barrel_heat_limit
                                                 //   热量上限, 0 ~ 511

    uint32_t shooter_barrel_cooling_rate  : 7;   // robot_status.shooter_barrel_cooling_value
                                                 //   冷却速率, 0 ~ 127
    uint8_t robot_id                      : 8;   // robot_status.robot_id,
    uint32_t rsvd                         : 8;   // 预留

    static constexpr uint32_t ID = board_comm_id::REFEREE_DATA;
} __attribute__((packed));
static_assert(sizeof(infantry2_c2g_referee_msg_t) == 8,
              "infantry2_c2g_referee_msg_t must be exactly 8 bytes");

// ---------------------------------------------------------------------------
// 工具函数
// ---------------------------------------------------------------------------

// 摇杆 float 归一化 -> int 定点 (带死区, 云台板发送侧使用)
inline int16_t rc_norm(float x, float deadzone = 0.02f) {
    if (x > -deadzone && x < deadzone)
        return 0;
    long v = (long)(x * CHASSIS_RC_SCALE);
    if (v >  CHASSIS_RC_SCALE)
        v =  CHASSIS_RC_SCALE;
    if (v < -CHASSIS_RC_SCALE)
        v = -CHASSIS_RC_SCALE;
    return (int16_t)v;
}

// 速度斜坡限幅: 当前值向目标值每次最多变化 max_step (底盘板接收侧使用)
inline float ramp_value(float cur, float target, float max_step) {
    float diff = target - cur;
    if (diff >  max_step) diff =  max_step;
    if (diff < -max_step) diff = -max_step;
    return cur + diff;
}

} // namespace pyro

#endif // __PYRO_BOARD_MSG_H__
