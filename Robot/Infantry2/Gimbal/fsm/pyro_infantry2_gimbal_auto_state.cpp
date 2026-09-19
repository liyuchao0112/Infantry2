#include "pyro_infantry2_gimbal.h"
#include <algorithm>

namespace pyro {

void infantry2_gimbal_t::fsm_active_t::state_auto_t::enter(owner *owner) {}

void infantry2_gimbal_t::fsm_active_t::state_auto_t::execute(owner *owner) {
    // 没识别到目标，自瞄会发送一个非法的值，此时不更新数据
    if (fabs(owner->_ctx.cmd->target_pitch_angle) <= pyro::PI)
        owner->_ctx.data.target_pitch_rad = owner->_ctx.cmd->target_pitch_angle;
    if (fabs(owner->_ctx.cmd->target_yaw_angle) <= pyro::PI)
        owner->_ctx.data.target_yaw_rad = owner->_ctx.cmd->target_yaw_angle;

    owner->_ctx.data.target_pitch_rad = std::clamp(owner->_ctx.data.target_pitch_rad,
        infantry2_gimbal::PITCH_MIN_MOTOR_RAD, infantry2_gimbal::PITCH_MAX_MOTOR_RAD);

    //角度超限
    const float yaw_error = owner->_ctx.data.target_yaw_rad - owner->_ctx.data.current_yaw_imu_rad;
    if (yaw_error > PI)
        owner->_ctx.data.target_yaw_rad -= 2.0f * PI;
    else if (yaw_error < -PI)
        owner->_ctx.data.target_yaw_rad += 2.0f * PI;

    _imu_control(&owner->_ctx);
    _send_motor_command(&owner->_ctx);
}

void infantry2_gimbal_t::fsm_active_t::state_auto_t::exit(owner *owner) {}

} // namespace pyro