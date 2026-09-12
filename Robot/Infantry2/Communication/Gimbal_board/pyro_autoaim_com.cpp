#include "pyro_autoaim_drv.h"
#include "pyro_board_msg.h"
#include "task.h"
#include "pyro_rc_base_drv.h"
#include "pyro_vt03_rc_drv.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_infantry2_gimbal.h"
#include "pyro_infantry2_booster.h"

using namespace pyro;

static TaskHandle_t autoaim_app_task;
static infantry2_autoaim_drv_t *autoaim_drv_ptr;

infantry2_autoaim_drv_t::rx_data_t autoaim_cmd;

extern infantry2_c2g_referee_msg_t referee_msg;

void autoaim_rxcmd(const infantry2_autoaim_drv_t::rx_data_t &rx_data) {
    autoaim_cmd = rx_data;
}

void autoaim_txdata(infantry2_autoaim_drv_t::tx_data_t &tx_data) {
    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();

    auto gimbal_ctx = infantry2_gimbal_t::instance()->get_ctx();
    auto booster_ctx = infantry2_booster_t::instance()->get_ctx();

    tx_data.curr_yaw = -gimbal_ctx.data.current_yaw_imu_rad;
    tx_data.curr_pitch = -gimbal_ctx.data.current_pitch_motor_rad;
    tx_data.self_v_magnitude = 0;
    tx_data.self_v_angle = 0;
    tx_data.state = 0;
    tx_data.autoaim = 1;
    tx_data.enemy_color = referee_msg.robot_id > 100;
    tx_data.curr_speed = infantry2_booster::TARGET_BULLET_SPEED * 100;
    tx_data.shoot_delay = 66;
    tx_data.stop_record = 0;
}

extern "C" {

    void infantry2_autoaim_app_thread(void *argument) {
        // 延时等待底层设备初始化完成
        vTaskDelay(pdMS_TO_TICKS(500));

        while (true) {
            if (autoaim_drv_ptr->check_online()) {
                const auto &rx_data = autoaim_drv_ptr->get_rx_data();
                autoaim_rxcmd(rx_data);
            }

            auto &tx_data = autoaim_drv_ptr->get_tx_data();
            autoaim_txdata(tx_data);
            autoaim_drv_ptr->send_data();

            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    void infantry2_autoaim_init() {
        // 1. 获取底层驱动实例
        autoaim_drv_ptr = &infantry2_autoaim_drv_t::get_instance();

        // 2. 启动驱动层的接收和解析任务
        autoaim_drv_ptr->start_rx();

        // 3. 创建应用层业务线程
        xTaskCreate(infantry2_autoaim_app_thread, "autoaim_app_thread", 256, nullptr,
                    configMAX_PRIORITIES - 3, &autoaim_app_task);

        // 4. 初始化完成，删除自身
        vTaskDelete(nullptr);
    }

}