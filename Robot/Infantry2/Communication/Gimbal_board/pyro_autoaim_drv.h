#ifndef __PYRO_AUTOAIM_DRV_H__
#define __PYRO_AUTOAIM_DRV_H__

#include "pyro_core_def.h"
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

    /**
     * @brief 帧 ID（写入帧头的消费者标识，与 usb_cdc_drv_t 的路由表对应）。
     * @note 上位机必须使用同样的编号，否则帧会被计入 rx_unknown_id 并丢弃。
     */
    static constexpr uint8_t RX_FRAME_ID = 0x01;   /**< 视觉 → 云台 */
    static constexpr uint8_t TX_FRAME_ID = 0x02;   /**< 云台 → 视觉 */

#if defined(AUTOAIM_USB_CDC)
    static infantry2_autoaim_drv_t &get_instance();
#endif

    void start_rx() const;
    tx_data_t &get_tx_data();
    status_t send_data() const;
    [[nodiscard]] const rx_data_t &get_rx_data() const;
    [[nodiscard]] bool check_online() const;
    [[nodiscard]] float get_com_interval() const;

  private:
    explicit infantry2_autoaim_drv_t();
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

    autoaim_task_t *_task;   // The internal task instance
    MessageBufferHandle_t _rx_msg_buf;

    tx_data_t _tx_payload{}; // 缓存用户修改的待发数据
    rx_data_t _latest_rx_data{};
    bool _is_online;

    // --- 新增：通信间隔与时间戳变量 ---
    float _com_interval_ms{0.0f};
    float _last_rx_time_ms{0.0f};

    void init_impl();
    void run_loop_impl();

    static bool rx_callback(const uint8_t *data, uint16_t len);

    void unpack(const rx_data_t *buf);
};

} // namespace pyro

#endif // __PYRO_AUTOAIM_DRV_H__