#include "pyro_infantry2_chassis.h"

namespace pyro {

void infantry2_chassis_t::fsm_active_t::state_follow_yaw_t::enter(owner *owner) {
    owner->_ctx.deps.pid.yaw_follow_pid->clear();
    owner->_ctx.data.target_yaw_rad = 0;
}

void infantry2_chassis_t::fsm_active_t::state_follow_yaw_t::execute(owner *owner) {
    _follow_yaw_solve(&owner->_ctx);
    _chassis_control(&owner->_ctx);
    _send_motor_command(&owner->_ctx);
}

void infantry2_chassis_t::fsm_active_t::state_follow_yaw_t::exit(owner *owner) {}

} // namespace pyro