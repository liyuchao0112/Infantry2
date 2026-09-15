#include "pyro_bsp_uart.h"
#include "pyro_bsp_can.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_vt03_rc_drv.h"
#include "pyro_dwt_drv.h"
#include "pyro_ins.h"
#include "pyro_referee.h"
#include "pyro_supercap_drv.h"
#include "pyro_usb_cdc_drv.h"

namespace pyro {

extern "C" {

    can_drv_t *can1_drv;
    can_drv_t *can2_drv;
    can_drv_t *can3_drv;
    ins_drv_t *ins_drv;

    void pyro_init_thread(void *argument) {
        // 1. 初始化 DWT (延时/计时)
        dwt_drv_t::init(480);

        // 2. 初始化所有 CAN 总线
        bsp_can::init_all();
        can1_drv = &bsp_can::get_can1();
        can2_drv = &bsp_can::get_can2();
        can3_drv = &bsp_can::get_can3();

        // 3. 初始化 IMU (BMI088)
        ins_drv = ins_drv_t::get_instance();
        ins_config_t ins_cfg;
        ins_cfg.calibrate = false;

        ins_cfg.direct = ins_config_t::imu_direct_t::DIRECT_1;
        ins_cfg.gx_offset = 0.00427565258;  // 陀螺仪零偏
        ins_cfg.gy_offset = -0.00199150061;
        ins_cfg.gz_offset = 0.000609153882;
        ins_cfg.g_norm = 9.85494423;
        ins_drv->init(ins_cfg);

#ifdef DR16_UART
        dr16_drv_t::instance().start();
        dr16_drv_t::instance().enable();
        DR16_UART.reset(100000, UART_WORDLENGTH_9B, UART_STOPBITS_2,
                        UART_PARITY_EVEN);
        DR16_UART.enable_rx_dma();
#endif

#ifdef VT03_UART
        vt03_drv_t::instance().start();
        vt03_drv_t::instance().enable();
        VT03_UART.reset(921600, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                UART_PARITY_NONE);
        VT03_UART.enable_rx_dma();
#endif

        rc_drv_t::init_virtual_rc();

#ifdef REFEREE_UART
        REFEREE_UART.reset(115200, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                            UART_PARITY_NONE);
        REFEREE_UART.enable_rx_dma();
        referee_drv_t::get_instance()->init();
#endif

#ifdef SUPERCAP_UART
        SUPERCAP_UART.reset(115200, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                            UART_PARITY_NONE);
        SUPERCAP_UART.enable_rx_dma();
        supercap_drv_t::get_instance()->start_rx();
#endif

#if defined(AUTOAIM_UART)
        AUTOAIM_UART.reset(921600, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                           UART_PARITY_NONE);
        AUTOAIM_UART.enable_rx_dma();
#elif defined(AUTOAIM_USB_CDC)
        // USB CDC 虚拟串口：启动设备栈（内部 tusb_init + tud_task 任务）。
        // 必须在调度器启动后被调用（本函数即为 FreeRTOS 任务），
        // 且需先于任何 tud_* 回调使用 instance()。
        usb_cdc_drv_t::instance().start();
        // 与 UART 路径保持对称的调用（USB 无链路参数，实现为 no-op）
        usb_cdc_drv_t::instance().reset(921600, 0, 0, 0);
        usb_cdc_drv_t::instance().enable_rx_dma();
#endif

        vTaskDelete(nullptr);
    }

}

} // namespace pyro