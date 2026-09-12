#include "pyro_autoaim_drv.h"
#include "pyro_infantry2_gimbal.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_rc_base_drv.h"
#include "pyro_vt03_rc_drv.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_bsp_can.h"
#include "pyro_board_com.h"
#include "pyro_infantry2_booster.h"

using namespace pyro;

static TaskHandle_t gimbal_task_handle = nullptr;

infantry2_gimbal_cmd_t *gimbal_cmd_ptr = nullptr;
infantry2_gimbal_deps_t *gimbal_deps_ptr = nullptr;
infantry2_gimbal_t *gimbal_ptr = nullptr;

// 板间遥控控制指令 (云台 -> 底盘), 直接使用协议结构体
static infantry2_g2c_cmd_t chassis_txcmd{};

infantry2_c2g_referee_msg_t referee_msg{};
bool is_referee_online = false;

static uint8_t chassis_seq = 0;

extern infantry2_autoaim_drv_t::rx_data_t autoaim_cmd;

void gimbal_deps_init() {
    gimbal_deps_ptr->motor.pitch = new dm_motor_drv_t(0x01, 0x00, bsp_can::can2);
    gimbal_deps_ptr->motor.yaw = new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_5, bsp_can::can1);

    static_cast<dm_motor_drv_t *>(gimbal_deps_ptr->motor.pitch)->set_position_range(-PI, PI);
    static_cast<dm_motor_drv_t *>(gimbal_deps_ptr->motor.pitch)
        ->set_rotate_range(infantry2_gimbal::PITCH_MIN_RADPS, infantry2_gimbal::PITCH_MAX_RADPS);
    static_cast<dm_motor_drv_t *>(gimbal_deps_ptr->motor.pitch)
        ->set_torque_range(infantry2_gimbal::PITCH_MIN_MOTOR_TORQUE, infantry2_gimbal::PITCH_MAX_MOTOR_TORQUE);

    //没写跟踪微分器，先空着
    
    //pid
    gimbal_deps_ptr->pid.yaw_pos_pid = new pid_t(15.5f, 0.0f, 0.0f, 1.0f, 10.0f, 50.0f, 1, 20.0f, 1, 4);
    gimbal_deps_ptr->pid.yaw_spd_pid = new pid_t(0.3f, 0.08f, 0.0003f, 0.0f, 10.0f, 50.0f, 1, 20.0f, 1, 4);
    gimbal_deps_ptr->pid.pitch_pos_pid = new pid_t(20.2f, 0.0004f, 0.006f, 0.4f, 9.0f, 50.0f, 1, 30.0f, 1, 4);
    gimbal_deps_ptr->pid.pitch_spd_pid = new pid_t(1.18f, 0.068f, 0.006f, 1.8f, 7.0f, 30.0f, 1, 15.0f, 1, 4);
}

void chassis_vt032cmd(uint32_t notify_val) {
    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();

    if (vrc.switches.gear.current_pos == pyro::sw_pos_t::UP) {
        chassis_txcmd.mode  = pyro::MODE_PASSIVE;
        chassis_txcmd.state = pyro::STATE_NORMAL;
        chassis_txcmd.vx = 0;
        chassis_txcmd.vy = 0;
        chassis_txcmd.wz = 0;
    } else if (vrc.switches.gear.current_pos == pyro::sw_pos_t::MID) {
        chassis_txcmd.mode  = pyro::MODE_ACTIVE;
        chassis_txcmd.state = pyro::STATE_FOLLOW_YAW;
        chassis_txcmd.vx = rc_norm(vrc.axes.ly);     // 前后
        chassis_txcmd.vy = rc_norm(vrc.axes.lx);     // 左右
        chassis_txcmd.wz = 0; // 选择由底盘根据yaw轴情况自动计算，实现跟随云台
    } else if (vrc.switches.gear.current_pos == pyro::sw_pos_t::DOWN) {
        // AUTO, 没写, 现在是PASSIVE
        chassis_txcmd.mode  = pyro::MODE_PASSIVE;
        chassis_txcmd.state = pyro::STATE_NORMAL;
        chassis_txcmd.vx = 0;
        chassis_txcmd.vy = 0;
        chassis_txcmd.wz = 0;
    }
}

void chassis_dr162cmd(uint32_t notify_val) {
    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();

    if (vrc.switches.right.current_pos == pyro::sw_pos_t::UP) {
        chassis_txcmd.mode  = pyro::MODE_PASSIVE;
        chassis_txcmd.state = pyro::STATE_NORMAL;
        chassis_txcmd.vx = 0;
        chassis_txcmd.vy = 0;
        chassis_txcmd.wz = 0;
    } else if (vrc.switches.right.current_pos == pyro::sw_pos_t::MID) {
        chassis_txcmd.mode  = pyro::MODE_ACTIVE;
        chassis_txcmd.state = pyro::STATE_FOLLOW_YAW;
        chassis_txcmd.vx = rc_norm(vrc.axes.ly);     // 前后
        chassis_txcmd.vy = rc_norm(vrc.axes.lx);     // 左右
        chassis_txcmd.wz = 0; // 选择由底盘根据yaw轴情况自动计算，实现跟随云台
    } else if (vrc.switches.right.current_pos == pyro::sw_pos_t::DOWN) {
        chassis_txcmd.mode  = pyro::MODE_ACTIVE;
        chassis_txcmd.state = pyro::STATE_SPIN;
        chassis_txcmd.vx = rc_norm(vrc.axes.ly);     // 前后
        chassis_txcmd.vy = rc_norm(vrc.axes.lx);     // 左右
        chassis_txcmd.wz = 0; // 选择由底盘根据yaw轴情况自动计算，实现跟随云台
    }
}

void gimbal_vt032cmd(uint32_t notify_val) {
    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();

    if(vrc.switches.gear.current_pos == pyro::sw_pos_t::UP) {
        gimbal_cmd_ptr->mode = infantry2_gimbal_cmd_t::mode_t::PASSIVE;
        gimbal_cmd_ptr->target_pitch_delta_angle = 0.0f;
        gimbal_cmd_ptr->target_yaw_delta_angle = 0.0f;
    }
    else if(vrc.switches.gear.current_pos == pyro::sw_pos_t::MID) {
        gimbal_cmd_ptr->mode = infantry2_gimbal_cmd_t::mode_t::ACTIVE;
        gimbal_cmd_ptr->state = infantry2_gimbal_cmd_t::state_t::MANUAL;
        gimbal_cmd_ptr->target_pitch_delta_angle = - vrc.axes.ry * infantry2_gimbal::RC_PITCH_COEFFICIENT;
        gimbal_cmd_ptr->target_yaw_delta_angle = vrc.axes.rx * infantry2_gimbal::RC_YAW_COEFFICIENT;
    }
    else if(vrc.switches.gear.current_pos == pyro::sw_pos_t::DOWN) {
        // AUTO, 没写, 现在是无力状态
        gimbal_cmd_ptr->mode = infantry2_gimbal_cmd_t::mode_t::PASSIVE;
        gimbal_cmd_ptr->target_pitch_delta_angle = 0.0f;
        gimbal_cmd_ptr->target_yaw_delta_angle = 0.0f;
    }
}

void gimbal_dr162cmd(uint32_t notify_val) {
    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();

    if(vrc.switches.right.current_pos == pyro::sw_pos_t::UP) {
        gimbal_cmd_ptr->mode = infantry2_gimbal_cmd_t::mode_t::PASSIVE;
        gimbal_cmd_ptr->target_pitch_delta_angle = 0.0f;
        gimbal_cmd_ptr->target_yaw_delta_angle = 0.0f;
    }
    else if(vrc.switches.right.current_pos == pyro::sw_pos_t::MID) {
        gimbal_cmd_ptr->mode = infantry2_gimbal_cmd_t::mode_t::ACTIVE;
        gimbal_cmd_ptr->state = infantry2_gimbal_cmd_t::state_t::MANUAL;
        gimbal_cmd_ptr->target_pitch_delta_angle = vrc.axes.ry * infantry2_gimbal::RC_PITCH_COEFFICIENT;
        gimbal_cmd_ptr->target_yaw_delta_angle = vrc.axes.rx * infantry2_gimbal::RC_YAW_COEFFICIENT;
    }
    else if(vrc.switches.right.current_pos == pyro::sw_pos_t::DOWN) {
        gimbal_cmd_ptr->mode = infantry2_gimbal_cmd_t::mode_t::ACTIVE;
        gimbal_cmd_ptr->state = infantry2_gimbal_cmd_t::state_t::AUTO;
        gimbal_cmd_ptr->target_pitch_angle = -autoaim_cmd.shoot_pitch;
        gimbal_cmd_ptr->target_yaw_angle = -autoaim_cmd.shoot_yaw;
    }
}

void gimbal_rxdata() {
#if REFEREE_EN
    board_com_t::instance().read(referee_msg);
    is_referee_online =
        !board_com_t::instance().is_stale<infantry2_c2g_referee_msg_t>(500);
#endif

}

extern "C" {

    void infantry2_gimbal_thread(void *argument) {
        while(true) {
            uint32_t notify_val = 0;

            xTaskNotifyWait(0x00, UINT32_MAX, &notify_val, 0);

#if GIMBAL_EN
            if (vt03_drv_t::instance().check_online())
                gimbal_vt032cmd(notify_val);
            else if (dr16_drv_t::instance().check_online())
                gimbal_dr162cmd(notify_val);
            else
                gimbal_cmd_ptr->mode = infantry2_gimbal_cmd_t::mode_t::PASSIVE;
#else
            gimbal_cmd_ptr->mode = infantry2_gimbal_cmd_t::mode_t::PASSIVE;
#endif

            gimbal_ptr->set_command(*gimbal_cmd_ptr);

#if CHASSIS_EN
            if(vt03_drv_t::instance().check_online()) {
                chassis_vt032cmd(notify_val);
            }
            else if(dr16_drv_t::instance().check_online()) {
                chassis_dr162cmd(notify_val);
            } else {
                chassis_txcmd.mode  = pyro::MODE_PASSIVE;
                chassis_txcmd.state = pyro::STATE_NORMAL;
                chassis_txcmd.vx = chassis_txcmd.vy = chassis_txcmd.wz = 0;
            }
#else
            chassis_txcmd.mode  = pyro::MODE_PASSIVE;
            chassis_txcmd.state = pyro::STATE_NORMAL;
            chassis_txcmd.vx = chassis_txcmd.vy = chassis_txcmd.wz = 0;
#endif

            chassis_txcmd.seq = ++chassis_seq;
            board_com_t::instance().send(chassis_txcmd);

            gimbal_rxdata();

            vTaskDelay(1);
        }
    }

    void infantry2_gimbal_init(void *argument) {
        gimbal_cmd_ptr = new infantry2_gimbal_cmd_t();
        gimbal_deps_ptr = new infantry2_gimbal_deps_t();
        gimbal_ptr = infantry2_gimbal_t::instance();

        gimbal_deps_init();
        gimbal_ptr->configure(*gimbal_deps_ptr);
        gimbal_ptr->start();

        xTaskCreate(infantry2_gimbal_thread, "infantry2_gimbal_thread", 256, nullptr,
                    configMAX_PRIORITIES - 1, &gimbal_task_handle);

        vTaskDelete(nullptr);
    }
}