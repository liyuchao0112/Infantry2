#include "pyro_board_com.h"

namespace pyro {

// ============================================================================
// 订阅清单: 新增板间消息只需在此处加一行 add<T>()
// ----------------------------------------------------------------------------
// !! 一个 CAN ID 在同一总线上只能注册一次 (register_rx_msg 对重复 ID 返回
//    PYRO_ERROR, 且旧 buffer 仍生效 -> 静默失效)。
//    因此应用层不得再自行 register_rx_msg 同一个 ID。
// ============================================================================
status_t board_com_t::init() {
#if BOARD == CHASSIS_BOARD
    // 0x100 云台 -> 底盘 遥控控制指令
    if (PYRO_OK != add<infantry2_g2c_cmd_t>())
        return PYRO_ERROR;

#elif BOARD == GIMBAL_BOARD
    // 0x101 底盘 -> 云台 裁判射击数据
    if (PYRO_OK != add<infantry2_c2g_referee_msg_t>())
        return PYRO_ERROR;
#endif

    return PYRO_OK;
}

status_t board_com_t::start() {
    xTaskCreate(com_thread, "pyro_board_com", 256, nullptr,
                configMAX_PRIORITIES - 2, nullptr);
    return PYRO_OK;
}

// ============================================================================
// 通用轮询: 遍历注册表解码, 与具体消息类型完全解耦
// ============================================================================
void board_com_t::com_thread(void* arg) {
    (void)arg;
    board_com_t& self = instance();

    while (true) {
        for (uint8_t i = 0; i < self._count; ++i) {
            table_entry_t& e = self._entries[i];

            if (!e.buffer->is_fresh())
                continue;

            std::array<uint8_t, 8> raw{};
            if (!e.buffer->get_data(raw))
                continue;

            e.decode(e.cache, raw.data());
            e.buffer->mark_read();
            *e.fresh = true;
        }

        vTaskDelay(1);   // 1 kHz
    }
}

} // namespace pyro

// ============================================================================
// 任务入口: 由 pyro_mission_planer 创建
// ============================================================================
extern "C" void board_com_init(void* argument) {
    (void)argument;

    pyro::board_com_t::instance().init();
    pyro::board_com_t::instance().start();

    vTaskDelete(nullptr);
}
