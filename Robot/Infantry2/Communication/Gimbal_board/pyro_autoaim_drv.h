#ifndef __PYRO_AUTOAIM_DRV_H__
#define __PYRO_AUTOAIM_DRV_H__

#include "pyro_core_def.h"
#include "pyro_uart_drv.h"
#include "pyro_task.h"
#include "message_buffer.h"
#include <cstdint>

namespace pyro {

class infantry2_autoaim_drv_t {
  public:
    struct tx_data_t {
        float       curr_yaw;
        float       curr_pitch;
        float       self_v_magnitude;
        float       self_v_angle;
        float       curr_speed;
        uint8_t     shoot_delay;
        uint8_t     state : 5;
        uint8_t     stop_record : 1;
        uint8_t     autoaim : 1;
        uint8_t     enemy_color : 1;
    } __attribute__((packed));

    struct rx_data_t {
        float       shoot_yaw;
        float       shoot_yaw_speed;
        float       shoot_yaw_acceleration;
        float       shoot_pitch;
        float       shoot_pitch_speed;
        float       shoot_pitch_acceleration;
        uint8_t     fire : 1;
        uint8_t     is_single_shot : 1;
        uint8_t     target_id : 6;
        uint8_t     aim_state;
    } __attribute__((packed));

#ifdef AUTOAIM_UART
    static infantry2_autoaim_drv_t &get_instance();
#endif

    void start_rx() const;
    tx_data_t &get_tx_data();
    status_t send_data() const;
    [[nodiscard]] const rx_data_t &get_rx_data() const;
    [[nodiscard]] bool check_online() const;
    [[nodiscard]] float get_com_interval() const;

  private:
    explicit infantry2_autoaim_drv_t(uart_drv_t *uart_handle);
    ~infantry2_autoaim_drv_t();

    class autoaim_task_t final : public task_base_t {
      public:
        explicit autoaim_task_t(infantry2_autoaim_drv_t *owner)
            : task_base_t("autoaim_task", 128, 256, priority_t::NORMAL), _owner(owner) {}

      protected:
        status_t init() override;
        void run_loop() override;

      private:
        infantry2_autoaim_drv_t *_owner;
    };

    struct frame_header_t {
        uint8_t sof;
    } __attribute__((packed));

    struct frame_tailer_t {
        uint16_t crc16;
        uint8_t end;
    } __attribute__((packed));

    struct rx_frame_tailer_t {
        uint16_t crc16;
    } __attribute__((packed));

    struct tx_packet_t {
        frame_header_t header;
        tx_data_t data;
        frame_tailer_t tailer;
    } __attribute__((packed));

    struct rx_packet_t {
        frame_header_t header;
        rx_data_t data;
        rx_frame_tailer_t tailer;
    } __attribute__((packed));

    uart_drv_t *_uart_drv;
    autoaim_task_t *_task;   // The internal task instance
    tx_packet_t *_tx_buffer; // DMA buffer
    MessageBufferHandle_t _rx_msg_buf;

    tx_data_t _tx_payload{}; // 缓存用户修改的待发数据
    rx_data_t _latest_rx_data{};
    bool _is_online;

    // --- 新增：通信间隔与时间戳变量 ---
    float _com_interval_ms{0.0f};
    float _last_rx_time_ms{0.0f};

    static constexpr uint8_t FRAME_SOF = 0xA5;

    void init_impl();
    void run_loop_impl();

    bool rx_callback(const uint8_t *p_data, uint16_t size, BaseType_t& xHigherPriorityTaskWoken) const;

    static status_t error_check(const rx_packet_t *buf);
    void unpack(const rx_packet_t *buf);
};

} // namespace pyro

#endif // __PYRO_AUTOAIM_DRV_H__