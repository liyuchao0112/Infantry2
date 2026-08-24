#include "pyro_infantry2_chassis.h"
#include "pyro_algo_common.h"

namespace pyro {

infantry2_chassis_t::infantry2_chassis_t()
        : module_base_t("infantry2_chassis", 512, 512, task_base_t::priority_t::HIGH) {
    _ctx.data = {};
}

status_t infantry2_chassis_t::_init() {
    _ctx.deps = _module_deps;
    return PYRO_OK;
}

void infantry2_chassis_t::_update_feedback() {
    //电机更新数据
    for(int i = 0; i < 4; i++) {
        _ctx.deps.motor.rudder[i]->update_feedback();
        _ctx.deps.motor.wheel[i]->update_feedback();
    }
    _ctx.deps.motor.yaw->update_feedback();

    //电机数据处理
    for(int i = 0; i < 4; i++) {
        _ctx.data.current_rudder_rad[i] =
            loop_fp32_constrain(
                _ctx.deps.motor.rudder[i]->get_current_position() - infantry2_chassis::RUDDER_MOTOR_OFFSET[i],
                -PI, PI);
        _ctx.data.current_rudder_radps[i] = _ctx.deps.motor.rudder[i]->get_current_rotate();

        _ctx.data.current_wheel_radps[i] = _ctx.deps.motor.wheel[i]->get_current_rotate();
    }
    _ctx.data.current_yaw_rad =
        loop_fp32_constrain(_ctx.deps.motor.yaw->get_current_position() - infantry2_chassis::YAW_MOTOR_OFFSET,
            -PI, PI);
    _ctx.data.current_yaw_radps = _ctx.deps.motor.yaw->get_current_rotate();
}

void infantry2_chassis_t::_fsm_execute() {
    _ctx.cmd = &_current_cmd;

    if (cmd_base_t::mode_t::PASSIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_passive_state);
    else if (cmd_base_t::mode_t::ACTIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_active_state);

    _main_fsm.execute(this);
}

void infantry2_chassis_t::_kinematics_solve(infantry2_chassis_ctx_t *ctx) {

}

void infantry2_chassis_t::_chassis_control(infantry2_chassis_ctx_t *ctx) {

}

void infantry2_chassis_t::_send_motor_command(infantry2_chassis_ctx_t *ctx) {
    for(int i = 0; i < 4; i++) {
        ctx->deps.motor.rudder[i]->send_torque(ctx->data.out_rudder_torque[i]);
        ctx->deps.motor.wheel[i]->send_torque(ctx->data.out_wheel_torque[i]);
    }
}

} // namespace pyro