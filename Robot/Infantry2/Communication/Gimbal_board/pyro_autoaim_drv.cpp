#include "pyro_autoaim_drv.h"

#if defined(AUTOAIM_USB_CDC)
#include "pyro_usb_cdc_drv.h"
#endif

#include "pyro_dwt_drv.h"

#include <cstring>

namespace pyro {

/* ============================== 内部任务 =============================== */

status_t infantry2_autoaim_drv_t::autoaim_task_t::init() {
    if (_owner == nullptr)
        return PYRO_ERROR;

    // 给 Idle 一个回收内存的窗口，不然栈空间不够
    vTaskDelay(pdMS_TO_TICKS(2));

    _owner->init_impl();
    return PYRO_OK;
}

void infantry2_autoaim_drv_t::autoaim_task_t::run_loop() {
    if (_owner)
        _owner->run_loop_impl();
}

#if defined(AUTOAIM_USB_CDC)
infantry2_autoaim_drv_t &infantry2_autoaim_drv_t::get_instance() {
    static infantry2_autoaim_drv_t instance;
    return instance;
}
#endif

infantry2_autoaim_drv_t::infantry2_autoaim_drv_t() {
    _task = new autoaim_task_t(this);
}

infantry2_autoaim_drv_t::~infantry2_autoaim_drv_t() {
    usb_cdc_drv_t::instance().remove_rx(RX_FRAME_ID);

    if (_task != nullptr) {
        _task->stop();
        delete _task;
        _task = nullptr;
    }

    if (_rx_msg_buf) {
        vMessageBufferDelete(_rx_msg_buf);
        _rx_msg_buf = nullptr;
    }
}

void infantry2_autoaim_drv_t::start_rx() const {
    if (_task)
        _task->start();
}

void infantry2_autoaim_drv_t::init_impl() {
    // 1. 消息缓冲：容量够放 ~4 帧（26 B/帧 + 长度前缀），用于跨任务传递载荷
    if (_rx_msg_buf == nullptr)
        _rx_msg_buf = xMessageBufferCreate(128);

    if (_rx_msg_buf == nullptr)
        return;

    // 2. 编译期校验：载荷必须能塞进 USB 驱动的单帧上限
    static_assert(sizeof(rx_data_t) <= usb_cdc::MAX_DATA_LENGTH,
                  "autoaim rx payload exceeds usb_cdc_drv_t::MAX_DATA_LEN");
    static_assert(sizeof(tx_data_t) <= usb_cdc::MAX_DATA_LENGTH,
                  "autoaim tx payload exceeds usb_cdc_drv_t::MAX_DATA_LEN");

    // 3. 把本模块挂到 USB 驱动的路由表上：收到 FRAME_ID == 0x01 的帧就回调
    (void) usb_cdc_drv_t::instance().register_rx(RX_FRAME_ID, infantry2_autoaim_drv_t::rx_callback);
}

void infantry2_autoaim_drv_t::run_loop_impl() {
    rx_data_t pkt{};

    while (true) {
        // 50 ms 无帧即判定视觉离线（上位机正常帧率远高于 20 Hz）
        const size_t n = xMessageBufferReceive(_rx_msg_buf, &pkt, sizeof(pkt),
                                               pdMS_TO_TICKS(50));

        if (n == sizeof(rx_data_t)) {
            const float now = dwt_drv_t::get_timeline_ms();

            // 只有"上一帧在线"时算出来的间隔才有意义，否则会把离线时长算进去
            if (_is_online)
                _com_interval_ms = now - _last_rx_time_ms;

            _last_rx_time_ms = now;
            _is_online       = true;

            unpack(&pkt);
        } else if (n == 0) {
            // 超时 → 上位机没在发（或视觉掉线）
            _is_online       = false;
            _com_interval_ms = 0.0f;
            _last_rx_time_ms = 0.0f;
        }
        // 其它长度说明缓冲里是半个包（理论上不会发生），丢弃继续等
    }
}

/* ISR Callback --------------------------------------------------------------*/
bool infantry2_autoaim_drv_t::rx_callback(uint8_t frame_id, const uint8_t *data, const uint16_t size) {
    // 长度不符说明上位机协议版本不匹配，直接丢（USB 驱动的 CRC 已保证字节可信）
    if (size != sizeof(rx_data_t))
        return false;

    // 运行在 usb_task 上下文，**不是 ISR**：必须用任务版 API。
    // 超时给 0：缓冲满了宁可丢这一帧，也绝不能阻塞 TinyUSB 栈。
    xMessageBufferSend(infantry2_autoaim_drv_t::get_instance()._rx_msg_buf, data, sizeof(rx_data_t), 0);
}

void infantry2_autoaim_drv_t::unpack(const rx_data_t *buf) {
    memcpy(&_latest_rx_data, &buf, sizeof(rx_data_t));
}

infantry2_autoaim_drv_t::tx_data_t &infantry2_autoaim_drv_t::get_tx_data() {
    return _tx_payload;
}

status_t infantry2_autoaim_drv_t::send_data() const {
    return usb_cdc_drv_t::instance().send(TX_FRAME_ID, (uint8_t*)&_tx_payload, sizeof(tx_data_t));
}

const infantry2_autoaim_drv_t::rx_data_t &infantry2_autoaim_drv_t::get_rx_data() const {
    return _latest_rx_data;
}

bool infantry2_autoaim_drv_t::check_online() const {
    return _is_online;
}

float infantry2_autoaim_drv_t::get_com_interval() const {
    return _com_interval_ms;
}

} // namespace pyro