#include "pyro_infantry2_chassis.h"
#include "pyro_bsp_can.h"
#include "pyro_board_com.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_referee.h"

using namespace pyro;

infantry2_chassis_cmd_t *chassis_cmd_ptr = nullptr;
infantry2_chassis_deps_t *chassis_deps_ptr = nullptr;
infantry2_chassis_t *chassis_ptr = nullptr;

static TaskHandle_t chassis_task_handle = nullptr;

// 目标速度 (来自云台, 归一化 x 最大速度) 与 当前输出速度 (斜坡限幅后)
static float tgt_vx = 0.0f, tgt_vy = 0.0f, tgt_wz = 0.0f;
static float cur_vx = 0.0f, cur_vy = 0.0f, cur_wz = 0.0f;

// 裁判数据转发分频计数 (1ms -> 20ms, 裁判系统本身约 10Hz)
static uint32_t referee_tx_cnt = 0;
static constexpr uint32_t REFEREE_TX_PERIOD_MS = 20;

void chassis_deps_init() {
    // ===== 电机 (按实际硬件核对型号 / CAN总线 / ID) =====
    // 舵电机
    chassis_deps_ptr->motor.rudder[0] =
        new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_3, bsp_can::can2); // FL
    chassis_deps_ptr->motor.rudder[1] =
        new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_1, bsp_can::can2); // FR
    chassis_deps_ptr->motor.rudder[2] =
        new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_4, bsp_can::can1); // BR
    chassis_deps_ptr->motor.rudder[3] =
        new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_2, bsp_can::can1); // BL

    // 轮电机
    chassis_deps_ptr->motor.wheel[0] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_3, bsp_can::can2); // FL
    chassis_deps_ptr->motor.wheel[1] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_1, bsp_can::can2); // FR
    chassis_deps_ptr->motor.wheel[2] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_4, bsp_can::can1); // BR
    chassis_deps_ptr->motor.wheel[3] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2, bsp_can::can1); // BL

    // 注意: yaw 电机需占用独立帧槽位, 与上方 wheel/rudder 错开, 按实际接线填写
    chassis_deps_ptr->motor.yaw =
        new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::register_id_t::id_5, bsp_can::can3);

    // ===== PID (按实际整定) =====
    for (int i = 0; i < 4; i++) {
        chassis_deps_ptr->pid.rud_pos_pid[i] =
            new pid_t(40.0f, 0.0f, 0.0f, 5.0f, 16.0f, 20.0f, 1, 0.0f, 0, 0);
        chassis_deps_ptr->pid.rud_spd_pid[i] =
            new pid_t(0.06f, 0.0f, 0.0f, 1.0f, 12.0f, 30.0f, 1, 0.0f, 0, 0);
        chassis_deps_ptr->pid.wheel_pid[i] =
            new pid_t(0.05f, 0.0f, 0.0f, 1.0f, 20.0f);
    }

    chassis_deps_ptr->pid.yaw_follow_pid =
        new pid_t(20.0f, 0.01f, 0.05f, 1.0f, 15.0f);
}

// 底盘板 <- 云台板: 读遥控指令快照, 心跳判定 -> 最大速度缩放 -> 斜坡限幅 -> 写 cmd
void chassis_rxcmd() {
    // 1) 读最新快照 (无新帧时也回填上次值), 返回值表示自上次读取后是否有更新
    pyro::infantry2_g2c_cmd_t rc{};
    const bool updated = pyro::board_com_t::instance().read(rc);

    if (updated) {
        tgt_vx = rc.vx / (float)pyro::CHASSIS_RC_SCALE * infantry2_chassis::MAX_VX;
        tgt_vy = rc.vy / (float)pyro::CHASSIS_RC_SCALE * infantry2_chassis::MAX_VY;
        tgt_wz = rc.wz / (float)pyro::CHASSIS_RC_SCALE * infantry2_chassis::MAX_WZ;

        chassis_cmd_ptr->mode  = (rc.mode == pyro::MODE_ACTIVE)
                                 ? cmd_base_t::mode_t::ACTIVE
                                 : cmd_base_t::mode_t::PASSIVE;
        chassis_cmd_ptr->state = (infantry2_chassis_cmd_t::state_t)rc.state;
    }

    // 2) 掉线保护: 超过阈值未收到新帧 -> 目标清零并切 PASSIVE
#if BOARD_COM_TIMEOUT_CHASSIS_CMD_ENABLE
    if (pyro::board_com_t::instance()
            .is_stale<pyro::infantry2_g2c_cmd_t>(BOARD_COM_TIMEOUT_CHASSIS_CMD_MS)) {
        tgt_vx = tgt_vy = tgt_wz = 0.0f;
        chassis_cmd_ptr->mode = cmd_base_t::mode_t::PASSIVE;
    }
#endif

    // 3) 加速度斜坡限幅 (防摇杆跳变急冲), 固定 1ms 控制周期
    const float dt = 0.001f;
    const float max_step = infantry2_chassis::MAX_ACCEL * dt;
    cur_vx = pyro::ramp_value(cur_vx, tgt_vx, max_step);
    cur_vy = pyro::ramp_value(cur_vy, tgt_vy, max_step);
    cur_wz = pyro::ramp_value(cur_wz, tgt_wz, max_step);

    chassis_cmd_ptr->vx = cur_vx;
    chassis_cmd_ptr->vy = cur_vy;
    chassis_cmd_ptr->wz = cur_wz;
}

// ============================================================================
// 底盘板 -> 云台板: 转发裁判系统数据 (0x101, 位域压缩单帧)
// 仅做数据搬运与定点化, 不含任何策略判断
// ============================================================================
void chassis_tx_referee() {
#ifdef REFEREE_UART
    pyro::referee_drv_t *ref = pyro::referee_drv_t::get_instance();
    if (nullptr == ref || !ref->is_online())
        return;

    const pyro::referee_data_t &d = ref->get_data();

    // 各字段按位宽饱和, 防止越界截断
    auto sat = [](uint32_t v, uint32_t max_v) -> uint32_t {
        return (v > max_v) ? max_v : v;
    };

    pyro::infantry2_c2g_referee_msg_t msg{};

    // 弹丸初速: float (m/s) -> x100 定点 (16bit 饱和)
    float spd = d.shoot.initial_speed;
    if (spd < 0.0f)
        spd = 0.0f;
    msg.initial_speed_x100 = sat(static_cast<uint32_t>(spd * 100.0f), 0xFFFFu);

    // 16bit 字段: 源即为 uint16, 饱和为恒等
    msg.shooter_17mm_barrel_heat = sat(d.power_heat.shooter_17mm_barrel_heat, 0xFFFFu);

    // 窄位宽字段: 按位宽饱和
    msg.shooter_barrel_heat_limit   = sat(d.robot_status.shooter_barrel_heat_limit, 0x1FFu);
    msg.shooter_barrel_cooling_rate = sat(d.robot_status.shooter_barrel_cooling_value, 0x7Fu);

    // 8bit 字段: 源即为 uint8
    msg.robot_id = sat(d.robot_status.robot_id, 0xFFu);

    pyro::board_com_t::instance().send(msg);
#endif
}

extern "C" {
    void infantry2_chassis_thread(void *argument) {
        while (true) {

#if CHASSIS_EN
            chassis_rxcmd();

            // 裁判数据转发 (1ms 分频到 20ms)
            if (++referee_tx_cnt >= REFEREE_TX_PERIOD_MS) {
                referee_tx_cnt = 0;
                chassis_tx_referee();
            }
#else
            chassis_cmd_ptr->mode = infantry2_chassis_cmd_t::mode_t::PASSIVE;
            chassis_cmd_ptr->vx = 0;
            chassis_cmd_ptr->vy = 0;
            chassis_cmd_ptr->wz = 0;
#endif

            chassis_ptr->set_command(*chassis_cmd_ptr);
            vTaskDelay(1);
        }
    }

    void infantry2_chassis_init(void *argument) {
        chassis_cmd_ptr = new infantry2_chassis_cmd_t();
        chassis_deps_ptr = new infantry2_chassis_deps_t();
        chassis_ptr = infantry2_chassis_t::instance();

        chassis_deps_init();
        chassis_ptr->configure(*chassis_deps_ptr);
        chassis_ptr->start();

        // 板间接收已由 board_com_t::init() 统一注册, 此处不再 register_rx_msg

        xTaskCreate(infantry2_chassis_thread, "infantry2_chassis_thread", 256, nullptr,
                    configMAX_PRIORITIES - 1, &chassis_task_handle);
        vTaskDelete(nullptr);
    }
}
