#include "pyro_infantry2_chassis.h"
#include <cmath>

namespace pyro {

void infantry2_chassis_t::fsm_active_t::state_spin_t::enter(owner *owner) {}

void infantry2_chassis_t::fsm_active_t::state_spin_t::execute(owner *owner) {
    float vx, vy;

    // yaw 相位延迟补偿
    float yaw_pred = owner->_ctx.data.current_yaw_rad + owner->_ctx.cmd->wz * 0.05f;

    vx = owner->_ctx.cmd->vx*cos(yaw_pred) + owner->_ctx.cmd->vy*sin(yaw_pred)
        - 0.05f*sinf((180.0f+40.0f)/180.0f*3.1415f);
    vy = owner->_ctx.cmd->vy*cos(yaw_pred) - owner->_ctx.cmd->vx*sin(yaw_pred)
        + 0.05f*cosf((180.0f+40.0f)/180.0f*3.1415f);

    owner->_ctx.data.target_states =
        _kinematics.solve(vx, vy, infantry2_chassis::SPIN_SPEED, owner->_ctx.data.current_states);
    _chassis_control(&owner->_ctx);
    _send_motor_command(&owner->_ctx);
}

void infantry2_chassis_t::fsm_active_t::state_spin_t::exit(owner *owner) {

}

} // namespace pyro