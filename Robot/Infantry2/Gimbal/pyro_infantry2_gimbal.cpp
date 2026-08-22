#include "pyro_infantry2_gimbal.h"
#include "pyro_ins.h"
#include "pyro_algo_common.h"
#include <algorithm>

namespace pyro {

infantry2_gimbal_t::infantry2_gimbal_t()
        : module_base_t("infantry2_gimbal", 512, 512, task_base_t::priority_t::HIGH) {
    _ctx.data = {};
}

status_t infantry2_gimbal_t::_init() {
    _ctx.deps = _module_deps;
    return PYRO_OK;
}

void infantry2_gimbal_t::_update_feedback() {
    //imu反馈数据
    ins_drv_t::get_instance()->get_rads_n(&_ctx.data.current_imu_yaw_rad,
        &_ctx.data.current_imu_pitch_rad, &_ctx.data.current_imu_roll_rad);
    ins_drv_t::get_instance()->get_gyro_n(&_ctx.data.current_imu_yaw_radps,
        &_ctx.data.current_imu_pitch_radps, &_ctx.data.current_imu_roll_radps);

    //电机反馈数据
    _ctx.deps.motor.pitch->update_feedback();
    _ctx.deps.motor.yaw->update_feedback();
    
    //电机数据处理
    _ctx.data.current_motor_pitch_rad =
        wrap2pi_f32(_ctx.deps.motor.pitch->get_current_position() - infantry2_gimbal::PITCH_MOTOR_OFFSET);
    _ctx.data.current_motor_pitch_radps =
        _ctx.deps.motor.pitch->get_current_rotate();
    
    _ctx.data.current_motor_yaw_rad =
        loop_fp32_constrain(_ctx.deps.motor.yaw->get_current_position(), -PI, PI);
    _ctx.data.current_motor_yaw_radps =
        _ctx.deps.motor.yaw->get_current_rotate();
}

void infantry2_gimbal_t::_fsm_execute() {
    _ctx.cmd = &_current_cmd;

    if (cmd_base_t::mode_t::PASSIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_passive_state);
    else if (cmd_base_t::mode_t::ACTIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_active_state);

    _main_fsm.execute(this);
}

void infantry2_gimbal_t::_mec_control(infantry2_gimbal_ctx_t *ctx) {
    //pitch位置环
    ctx->data.target_pitch_radps =
        ctx->deps.pid.pitch_pos_pid->calculate(
            ctx->data.target_pitch_rad, ctx->data.current_motor_pitch_rad);
    
    //pitch速度环
    ctx->data.out_pitch_torque =
        ctx->deps.pid.pitch_spd_pid->calculate(
            ctx->data.target_pitch_radps, ctx->data.current_motor_pitch_radps)
        + ctx->data.gravity_compensate;
    
    ctx->data.out_pitch_torque = std::clamp(ctx->data.out_pitch_torque,
        infantry2_gimbal::PITCH_MIN_MOTOR_TORQUE, infantry2_gimbal::PITCH_MAX_MOTOR_TORQUE);

    //yaw位置环
    ctx->data.target_yaw_radps =
        ctx->deps.pid.yaw_pos_pid->calculate(
            ctx->data.target_yaw_rad, ctx->data.current_motor_yaw_rad);
    
    //yaw速度环
    ctx->data.out_yaw_torque =
        ctx->deps.pid.yaw_spd_pid->calculate(
            ctx->data.target_yaw_radps, ctx->data.current_motor_yaw_radps);

    ctx->data.out_yaw_torque = std::clamp(ctx->data.out_yaw_torque,
        infantry2_gimbal::YAW_MIN_MOTOR_TORQUE, infantry2_gimbal::YAW_MAX_MOTOR_TORQUE);
}

} // namespace pyro