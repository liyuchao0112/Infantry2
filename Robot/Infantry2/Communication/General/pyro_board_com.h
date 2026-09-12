#ifndef __PYRO_BOARD_COMM_H__
#define __PYRO_BOARD_COMM_H__

#include "pyro_bsp_can.h"
#include "pyro_core_def.h"

#include "pyro_board_msg.h"
#include "infantry2_config.h"    // BOARD_COMM_CAN / BOARD_COMM_TIMEOUT_* 配置

#include "FreeRTOS.h"
#include "task.h"

#include <array>
#include <cstring>
#include <cstdint>

namespace pyro {

// ============================================================================
// rx_slot_t<T> —— 每个消息类型一个静态订阅槽
// ----------------------------------------------------------------------------
// 持有: ISR 写入的 can_msg_buffer_t + 线程解码缓存 + 待消费标志。
// 按类型静态唯一 (Meyers singleton)。
// !! 编译选项含 -fno-threadsafe-statics, 首次调用须落在单线程初始化阶段。
// ============================================================================
template <typename T>
class rx_slot_t {
public:
    static rx_slot_t& get() {
        static rx_slot_t instance;
        return instance;
    }

    rx_slot_t(const rx_slot_t&) = delete;
    rx_slot_t& operator=(const rx_slot_t&) = delete;

    /** @brief 在指定总线注册接收 (仅 board_comm_t::add<T>() 调用) */
    status_t attach(bsp_can::which_can bus) {
        can_drv_t* can = bsp_can::get_can(bus);
        if (nullptr == can)
            return PYRO_PARAM_ERROR;
        return can->register_rx_msg(&_buffer);
    }

    /** @brief 读取最新快照; 返回自上次读取后是否有更新。
     *         无论是否有更新, 都把最新缓存写回 out。 */
    bool read(T& out) {
        taskENTER_CRITICAL();
        out = _cache;
        const bool updated = _fresh;
        _fresh = false;
        taskEXIT_CRITICAL();
        return updated;
    }

    /** @brief 距上次收到数据是否已超时 (从未收到也视为超时) */
    bool is_stale(TickType_t timeout_ms) const {
        return (xTaskGetTickCount() - _buffer.get_last_update_time()) >=
               pdMS_TO_TICKS(timeout_ms);
    }

    /* ---- 供 board_comm_t 组装类型擦除表项 ---- */
    can_msg_buffer_t* buffer() { return &_buffer; }
    void*             cache()  { return &_cache;  }
    volatile bool*    fresh()  { return &_fresh;  }

    static void decode(void* dst, const uint8_t* src) {
        std::memcpy(dst, src, sizeof(T));
    }

private:
    rx_slot_t() : _buffer(T::ID) {}

    can_msg_buffer_t _buffer;
    T                _cache{};
    volatile bool    _fresh{false};
};

// ============================================================================
// board_comm_t —— 板间 CAN 通信管理器 (单例)
// ----------------------------------------------------------------------------
// 负责注册 RX、启动通信线程、统一轮询解码;
// 对外提供类型化的 send<T>() / read<T>() / is_stale<T>()。
// ============================================================================
class board_com_t {
public:
    static board_com_t& instance() {
        static board_com_t inst;
        return inst;
    }

    board_com_t(const board_com_t&) = delete;
    board_com_t& operator=(const board_com_t&) = delete;

    /** @brief 按 BOARD 注册本板订阅清单 */
    status_t init();

    /** @brief 创建通信线程 */
    status_t start();

    /** @brief 注册接收。T 需带 static constexpr uint32_t ID */
    template <typename T>
    status_t add() {
        // 重复 ID 检测: register_rx_msg 对重复 ID 返回错误且旧 buffer 仍生效
        for (uint8_t i = 0; i < _count; ++i) {
            if (_entries[i].id == T::ID)
                return PYRO_PARAM_ERROR;
        }
        if (_count >= MAX_SLOTS)
            return PYRO_NO_MEMORY;

        rx_slot_t<T>& slot = rx_slot_t<T>::get();
        const status_t st = slot.attach(BOARD_COMM_CAN);
        if (PYRO_OK != st)
            return st;

        _entries[_count].id     = T::ID;
        _entries[_count].buffer = slot.buffer();
        _entries[_count].cache  = slot.cache();
        _entries[_count].fresh  = slot.fresh();
        _entries[_count].decode = &rx_slot_t<T>::decode;
        ++_count;

        return PYRO_OK;
    }

    /** @brief 发送。T 需带 static constexpr uint32_t ID */
    template <typename T>
    status_t send(const T& msg) const {
        static_assert(sizeof(T) <= 8, "board msg must fit one CAN frame");

        can_drv_t* can = bsp_can::get_can(BOARD_COMM_CAN);
        if (nullptr == can)
            return PYRO_PARAM_ERROR;

        uint8_t data[8]{};              // 零填充, 短结构体尾部为 0
        std::memcpy(data, &msg, sizeof(T));
        return can->send_msg(T::ID, data);
    }

    /** @brief 读取最新快照 (未订阅的类型恒返回 false) */
    template <typename T>
    bool read(T& out) {
        return rx_slot_t<T>::get().read(out);
    }

    /** @brief 失联判定 (T 需为本板订阅过的类型) */
    template <typename T>
    bool is_stale(TickType_t timeout_ms) {
        return rx_slot_t<T>::get().is_stale(timeout_ms);
    }

private:
    board_com_t() = default;

    static constexpr uint8_t MAX_SLOTS = 12;

    /** @brief 类型擦除表项: 通信线程据此通用轮询 */
    struct table_entry_t {
        uint32_t          id;
        can_msg_buffer_t* buffer;
        void*             cache;
        volatile bool*    fresh;
        void            (*decode)(void*, const uint8_t*);
    };

    table_entry_t _entries[MAX_SLOTS]{};
    uint8_t       _count{0};

    static void com_thread(void* arg);
};

} // namespace pyro

extern "C" void board_com_init(void* argument);

#endif // __PYRO_BOARD_COMM_H__