#include "pyro_infantry2_gimbal.h"

namespace pyro {

void infantry2_gimbal_t::fsm_active_t::on_enter(owner *owner) {
    //防跳变
    if(owner->_ctx.cmd->is_imu_control) {
        owner->_ctx.data.target_pitch_rad = owner->_ctx.data.current_pitch_imu_rad;
        owner->_ctx.data.target_yaw_rad = owner->_ctx.data.current_yaw_imu_rad;
    } else {
        owner->_ctx.data.target_pitch_rad = owner->_ctx.data.current_pitch_motor_rad;
        owner->_ctx.data.target_yaw_rad = owner->_ctx.data.current_yaw_motor_rad;
    }

    
}

void infantry2_gimbal_t::fsm_active_t::on_execute(owner *owner) {
    
}

void infantry2_gimbal_t::fsm_active_t::on_exit(owner *owner) {
    
}

};