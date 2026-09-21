# USB 虚拟串口（TinyUSB CDC）实现方案

> **目标**：使用 TinyUSB 实现 CDC-ACM 虚拟串口，以 USB 模拟 UART 与自瞄通信；**调用层无感**，USB 与普通串口（UART）的差异只在驱动层区分。
>
> **范围**：阶段 1-2（USB 底座 + 枚举/回环验证）给出完整可落地实现；阶段 3（接入自瞄业务）只给概要。
>
> **约束**：本方案**不修改 `bugs.md` 中记录的任何既有问题**（含 §1 `referee_msg` 链接冲突、§2 底盘板编译、§5 自瞄状态未接线、§8 `vPortDmaFree` 等），这些属于独立议题，本方案既不依赖也不触发对它们的修改。

> **改进记录**：A/B 类评审改进已就地合入本文档（代码/配置的修正已体现在对应小节），设计约束与决策依据汇总见 **§8**。
>
> **通用化**：USB CDC 驱动的通用化（去除机型指纹 + 对齐 BSP 约定）方案见 **§9**（已定稿，待实施，尚未改动代码）。

---

## 1. 现状（已核对，含证据）

| 项 | 结论 | 证据 |
|---|---|---|
| MCU / 工具链 | STM32H723，GCC + CMake + FreeRTOS，`project(PYRo_Robot)` | `CMakeLists.txt:19` |
| 板级选择 | `Robot/Infantry2/CMakeLists.txt` 内 `set(BOARD "GIMBAL_BOARD")` | `Robot/Infantry2/CMakeLists.txt:4` |
| TinyUSB | `third_party/TinyUSB`，版本 **0.21.0**，与 STM32H723 匹配 | `third_party/TinyUSB/src/tusb_option.h:13-15` |
| TinyUSB 接入状态 | **尚未接入任何 CMakeLists**（全仓检索仅其自身内部出现） | 全仓检索结果 |
| USB 硬件 | 仅 `USB1_OTG_HS`(0x40040000)，`USB2_OTG_FS` 未定义；CubeMX 配为内置 FS PHY、DMA 关闭；**USB 时钟源 = HSI48** | `CubeMX/Core/Src/usb_otg.c:41-51,79-80`、`main.c:188-190`、`stm32h723xx.h:2132,2686` |
| 中断 | 向量表仅 `OTG_HS_IRQHandler`（`OTG_HS_IRQn=77`），现内容为 `HAL_PCD_IRQHandler`，优先级 5 | `stm32h7xx_it.c:478-487`、`usb_otg.c:89` |
| USB 引脚 | `gpio.c` 中**未配置** PA11/PA12 | `CubeMX/Core/Src/gpio.c:42-75` |
| TinyUSB 移植层 | H7 → `TUP_USBIP_DWC2`；有 `stm32h723nucleo` 现成参考（单口当 FS、PA11/PA12 用 AF10） | `src/common/tusb_mcu.h:296`、`hw/bsp/stm32h7/boards/stm32h723nucleo/board.h:50-52` |
| 速度开关（关键） | `TUD_OPT_HIGH_SPEED = CFG_TUD_MAX_SPEED & OPT_MODE_HIGH_SPEED`；非 HS 时走 FS PHY 分支并置 `GCCFG.PWRDWN`（开启内置 FS PHY） | `src/tusb_option.h:502`、`dwc2_common.c:42-66`、`dwc2_stm32.h:178-203` |
| dwc2 工作模式 | 默认 `CFG_TUD_DWC2_DMA_ENABLE=0 / SLAVE_ENABLE=1`，与 CubeMX `dma_enable=DISABLE` 一致，**无 dcache/对齐陷阱** | `src/tusb_option.h:337-351`、`tusb_mcu.h:490` |
| 自瞄通信层依赖 | 仅依赖 `uart_drv_t*` 的 5 个方法：`reset / write(p,size) / enable_rx_dma / add_rx_event_callback / remove_rx_event_callback`；收/发单帧 ≈ **29B / 26B** | `pyro_autoaim_drv.h:52,93`、`.cpp:41,85,133,174` |
| UART 额外能力使用面 | `uart_drv_t::state.*` 全仓 **0 处读取**；`set_tx_cplt_callback` 仅裁判系统使用（自瞄不用） | 全仓检索结果 |
| 框架任务基类 | `task_base_t(name, init_stack, loop_stack, priority)`；`start()` 同步执行 `init()` 后创建 loop 任务 | `PYRo/Core/Task/pyro_task.h:49-56`、`pyro_task.cpp:36-90` |

---

## 2. 分层契约（"调用层无感"是硬约束）

```
调用层（零 #ifdef、零分支）
  pyro_autoaim_com.cpp / infantry2_autoaim_drv_t
  只用：write / rx 回调（= 完整一帧）/ 超时在线判定
        │
        ▼
  pyro::serial_itf_t（唯一契约，纯虚接口：只含消费者经指针调用的方法）
        ├── uart_drv_t      : 波特率真实生效（reset + enable_rx_dma）、ISR 回调、IDLE 分帧
        └── usb_cdc_drv_t   : start() + enable_rx()、任务上下文回调、★内部组帧、DTR/挂载
              ↑ 所有 "USB ≠ UART" 的差异全部关在这一层及以下，不向上泄漏
```

**契约三条**（驱动层必须对调用层兑现）：

1. **一次 rx 回调 = 一个完整帧**：USB 侧由 `usb_cdc_drv_t` 内部 `frame_parser_t` 组帧保证；UART 侧由 IDLE 语义天然接近（并可选启用同一组帧器）。
2. **`write()` 返回 `PYRO_BUSY` 表示"没发出去、可重试"**：USB 与 UART 语义一致。
3. **"在线"判定由调用层超时逻辑负责**（沿用现有 50ms 超时），驱动层只提供物理收发能力。

---

## 3. 阶段 1-2：USB 底座 + 枚举/回环验证

### 3.1 文件清单

**新增：**

```
PYRo/Peripheral/Serial/pyro_serial_itf.h        # 接口（本阶段定稿，阶段 3 不再改）
PYRo/Peripheral/Serial/pyro_frame_parser.h      # 组帧器（放驱动层，调用层无感的关键）
PYRo/Peripheral/USB/tusb_config.h               # TinyUSB 配置
PYRo/Peripheral/USB/pyro_usb_descriptors.c      # 描述符
PYRo/Peripheral/USB/pyro_usb_cdc_drv.h          # 驱动声明
PYRo/Peripheral/USB/pyro_usb_cdc_drv.cpp        # 驱动实现 + USB 任务 + 中断桥接 pyro_usb_irq_handler()
```

**修改（4 处，约 20 行）：**

```
CMakeLists.txt                        新增 tinyusb target（置于 add_subdirectory(PYRo) 之前）
PYRo/CMakeLists.txt                   增源文件 / 头文件路径 / 链接 tinyusb
CubeMX/Core/Src/main.c                注释掉 MX_USB_OTG_HS_PCD_Init()
CubeMX/Core/Src/stm32h7xx_it.c/.h     迁移 OTG_HS_IRQHandler 到 USB 驱动
Robot/Infantry2/pyro_init_thread.cpp  临时启动 USB（仅测试宏下生效）
```

**不动（保证零回归）：**

```
PYRo/Peripheral/UART/**                             （阶段 3 才加基类）
Robot/Infantry2/Communication/Gimbal_board/**       （阶段 3 才改指针类型）
Robot/Infantry2/CMakeLists.txt 的 AUTOAIM_UART=PYRO_UART7
bugs.md                                             （本方案不涉及其中任何问题）
```

### 3.2 `PYRo/Peripheral/Serial/pyro_serial_itf.h`（新增，完整）

```cpp
#ifndef __PYRO_SERIAL_ITF_H__
#define __PYRO_SERIAL_ITF_H__

#include "pyro_core_def.h"
#include "FreeRTOS.h"
#include <cstdint>
#include <functional>

namespace pyro
{

/**
 * @brief "字节流链路"抽象接口：UART 与 USB-CDC 的统一契约。
 *
 * 契约（驱动层必须对调用层兑现，调用层不得感知链路类型）：
 *  1) 一次 rx 回调 = 一个完整帧（由驱动层内部组帧保证，见 set_frame_config）；
 *  2) write() 返回 PYRO_BUSY 表示"没发出去，可重试"；
 *  3) "在线"判定由调用层超时逻辑负责，驱动层只提供物理收发能力。
 */
class serial_itf_t
{
  public:
    /**
     * @brief 接收回调。p 为字节流（若已 set_frame_config，则一次调用恰为一整帧）。
     * @note 调用上下文由实现决定：
     *       uart_drv_t    -> DMA/IDLE 中断（ISR）
     *       usb_cdc_drv_t -> TinyUSB 设备任务（tud_task）
     *       实现在任务上下文时会传入 woken = pdFALSE，回调内不得调用 portYIELD_FROM_ISR。
     * @warning p 指向驱动内部缓冲，**仅在本次回调期间有效**，回调方必须自行拷贝
     *          （现有调用层是 memcpy 进 MessageBuffer，满足该约束，见 §8.4）。
     *          回调内也不得做耗时/阻塞操作：USB 实现下会拖慢整个 USB 栈。
     */
    using rx_event_func = std::function<bool(
        uint8_t *p, uint16_t size, BaseType_t &xHigherPriorityTaskWoken)>;

    virtual ~serial_itf_t() = default;

    /**
     * @brief 非阻塞写。
     * @note 只允许在任务上下文调用，**禁止在 ISR 中调用**（USB 实现内部会走 TinyUSB FIFO
     *       的 osal 互斥；跨任务调用是安全的，见 §8.2）。
     * @return PYRO_OK=已入队；PYRO_BUSY=缓冲不足可重试；PYRO_ERROR=链路不可用（未挂载等）。
     */
    virtual status_t write(const uint8_t *p, uint16_t size) = 0;

    /** @brief 阻塞写（带超时）。 */
    virtual status_t write(const uint8_t *p, uint16_t size, uint32_t waittime) = 0;

    /** @brief 注册接收回调（owner 用于注销，通常传 this）。 */
    virtual void add_rx_event_callback(const rx_event_func &func,
                                       uint32_t owner) = 0;

    /** @brief 按 owner 注销接收回调。 */
    virtual status_t remove_rx_event_callback(uint32_t owner) = 0;

    /**
     * @brief 配置驱动层组帧：一次 rx 回调保证给到 [SOF + frame_len] 的完整帧。
     * @param sof       帧头字节（如 0xA5）
     * @param frame_len 整帧长度（含帧头与校验）
     * @note 传 sof = 0 关闭组帧（透传字节流），默认关闭。
     *       uart_drv_t 的实现为 no-op（保持现有 IDLE 语义不变）。
     */
    virtual void set_frame_config(uint8_t sof, uint16_t frame_len) = 0;
};

} // namespace pyro

#endif // __PYRO_SERIAL_ITF_H__
```

---

### 3.3 `PYRo/Peripheral/Serial/pyro_frame_parser.h`（新增，完整）

```cpp
#ifndef __PYRO_FRAME_PARSER_H__
#define __PYRO_FRAME_PARSER_H__

#include <cstdint>

namespace pyro
{

/**
 * @brief 定长帧切分器（字节流 -> 完整帧）。
 *
 * 用法：把每个字节喂入 feed()，内部扫描 SOF，凑满 frame_len 字节即回调一次 on_frame。
 *
 * 特性：
 *  - 纯数据、无锁、无动态分配；单帧缓冲固定 MAX_FRAME_LEN。
 *  - 只在同一上下文被访问（UART ISR 或 USB 任务，二者互斥），故无需加锁。
 *  - 不做内容校验（CRC 由上层业务校验），本类只负责定界。
 */
class frame_parser_t
{
  public:
    static constexpr uint16_t MAX_FRAME_LEN = 64; // 自瞄帧 29B，留足余量

    /** @brief 置位 SOF 与帧长。frame_len 必须 <= MAX_FRAME_LEN。 */
    void configure(uint8_t sof, uint16_t frame_len)
    {
        _sof = sof;
        _len = (frame_len == 0 || frame_len > MAX_FRAME_LEN) ? 0 : frame_len;
        _idx = 0;
    }

    /** @brief 丢弃半帧、重新同步（掉线/拔插时调用）。 */
    void reset() { _idx = 0; }

    /** @brief 是否已配置帧长。 */
    [[nodiscard]] bool enabled() const { return _len != 0; }

    /**
     * @brief 喂入一段字节流，每凑满一帧调用一次 on_frame(frame, len)。
     * @param on_frame 返回 false 可提前中止本次喂入。
     * @return 本次产生的完整帧数量。
     */
    template <typename F>
    uint16_t feed(const uint8_t *p, uint16_t size, F &&on_frame)
    {
        uint16_t produced = 0;
        for (uint16_t i = 0; i < size; ++i)
        {
            const uint8_t b = p[i];
            if (_idx == 0 && b != _sof)
                continue;              // 帧外字节：丢弃
            _buf[_idx++] = b;
            if (_idx == _len)
            {
                _idx = 0;
                ++produced;
                if (!on_frame(_buf, _len))
                    break;
            }
        }
        return produced;
    }

  private:
    uint8_t  _sof{0xA5};
    uint16_t _len{0};
    uint16_t _idx{0};
    uint8_t  _buf[MAX_FRAME_LEN]{};
};

} // namespace pyro

#endif // __PYRO_FRAME_PARSER_H__
```

---

### 3.4 `PYRo/Peripheral/USB/tusb_config.h`（新增，完整）

```c
#ifndef TUSB_CONFIG_H_
#define TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------
// MCU / OS（CFG_TUSB_MCU、CFG_TUSB_OS 由 CMake 传入，此处只校验）
//--------------------------------------------------------------------
#if !defined(CFG_TUSB_MCU)
#error CFG_TUSB_MCU must be defined by the build system
#endif

#define CFG_TUSB_DEBUG            0
#define CFG_TUD_ENABLED           1

//--------------------------------------------------------------------
// ★ STM32H723：只有 USB1_OTG_HS + 内置 FS PHY
//   CFG_TUD_MAX_SPEED 必须是 FULL：
//     -> TUD_OPT_HIGH_SPEED = 0
//     -> dwc2_core_is_highspeed_phy() = false
//     -> phy_fs_init() -> dwc2_phy_init(PHY_NOT_SUPPORTED) -> GCCFG.PWRDWN = 1（开内置 FS PHY）
//   若误设为 HIGH_SPEED，TinyUSB 会清掉 PWRDWN 导致完全不枚举。
//--------------------------------------------------------------------
#define CFG_TUD_MAX_SPEED         OPT_MODE_FULL_SPEED
// 说明：TUD_OPT_HIGH_SPEED 只由 CFG_TUD_MAX_SPEED 决定（src/tusb_option.h:502）；
//       BOARD_TUD_MAX_SPEED 仅为兼容 board.mk 惯例保留，在 STM32 dwc2 端口上不生效。
#define BOARD_TUD_MAX_SPEED       OPT_MODE_FULL_SPEED

// ★ 必须与 pyro_usb_cdc_drv.cpp 中 tusb_int_handler(BOARD_TUD_RHPORT, true) 一致
#ifndef BOARD_TUD_RHPORT
#define BOARD_TUD_RHPORT          1
#endif

#define CFG_TUD_ENDPOINT0_SIZE    64

//--------------------------------------------------------------------
// Class
//--------------------------------------------------------------------
#define CFG_TUD_CDC               1
#define CFG_TUD_MSC               0
#define CFG_TUD_HID               0
#define CFG_TUD_MIDI              0
#define CFG_TUD_VENDOR            0
#define CFG_TUD_CDC_NOTIFY        1   // 与 TUD_CDC_DESCRIPTOR 中的 notify 端点对应

// CDC FIFO（单帧仅 ~29B，给足余量便于吸收突发）
#define CFG_TUD_CDC_RX_BUFSIZE    1024
#define CFG_TUD_CDC_TX_BUFSIZE    1024
#define CFG_TUD_CDC_RX_EPSIZE     64
#define CFG_TUD_CDC_TX_EPSIZE     64

//--------------------------------------------------------------------
// TX 覆盖行为与 DTR 策略（取舍与源码依据见 §8.1）
//   默认 1：DTR=0（上位机未打开端口）时 TX FIFO 可被覆盖，tud_cdc_write() 不会失败，
//           即"设备以为在发、其实无人接收"，上层只能靠 50ms 无收包超时感知。
//   若阶段 1-2 实测确认上位机必然置 DTR，可改为 0 并让 write() 依赖 tud_cdc_connected()，
//   使"发送失败"能被上层立刻观测到。
//--------------------------------------------------------------------
#define CFG_TUD_CDC_TX_OVERWRITABLE_IF_NOT_CONNECTED 1

// slave 模式（默认），与 CubeMX dma_enable = DISABLE 一致；无需 dcache 维护
#define CFG_TUSB_MEM_ALIGN        __attribute__ ((aligned(4)))

#ifdef __cplusplus
}
#endif
#endif /* TUSB_CONFIG_H_ */
```

> 无需调用 `tud_configure(1, TUD_CFGID_DWC2, ...)`：默认 `vbus_sensing = false`，正好对应本板无 VBUS 分压（CubeMX 中 `vbus_sensing_enable = DISABLE`，TinyUSB 会强制 B-valid 并使能 `NOVBUSSENS`）。

---

### 3.5 `PYRo/Peripheral/USB/pyro_usb_descriptors.c`（新增，完整）

```c
#include "tusb.h"
#include <string.h>

//--------------------------------------------------------------------+
// Device Descriptor
//--------------------------------------------------------------------+
#define USB_VID   0xCAFE   // TODO: 待确认为本项目正式 VID
#define USB_PID   0x4010   // TODO: 待确认为本项目正式 PID
#define USB_BCD   0x0100

tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = USB_BCD,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01,
};

// Invoked when received GET DEVICE DESCRIPTOR
uint8_t const *tud_descriptor_device_cb(void) {
    return (uint8_t const *) &desc_device;
}

//--------------------------------------------------------------------+
// Configuration Descriptor（单 CDC；FS-only，无需 HS / OTHER_SPEED）
//--------------------------------------------------------------------+
enum {
    ITF_NUM_CDC = 0,
    ITF_NUM_CDC_DATA,
    ITF_NUM_TOTAL
};

#define EPNUM_CDC_NOTIF   0x81
#define EPNUM_CDC_OUT     0x02
#define EPNUM_CDC_IN      0x82

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

static uint8_t const desc_fs_configuration[] = {
    // Config number, interface count, string index, total length, attribute, power(mA)
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    // Interface number, string index, EP notif addr & size, EP out, EP in, EP size
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8,
                       EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
};

// Invoked when received GET CONFIGURATION DESCRIPTOR
uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void) index;
    return desc_fs_configuration;
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+
enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
    STRID_CDC_ITF
};

static char const *string_desc_arr[] = {
    (const char[]) { 0x09, 0x04 },   // 0: English (0x0409)
    "PYRo",                          // 1: Manufacturer
    "Infantry2 Virtual COM",         // 2: Product
    "PYRO-INF2-0001",                // 3: Serial
    "PYRo CDC",                      // 4: CDC interface
};

static uint16_t _desc_str[32 + 1];

// Invoked when received GET STRING DESCRIPTOR
uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void) langid;
    uint8_t chr_count;

    if (index == 0) {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else {
        if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) {
            return NULL;
        }
        const char *str = string_desc_arr[index];
        chr_count = (uint8_t) strlen(str);
        if (chr_count > 31) {
            chr_count = 31;
        }
        for (uint8_t i = 0; i < chr_count; i++) {
            _desc_str[1 + i] = str[i];   // ASCII -> UTF-16LE
        }
    }

    _desc_str[0] = (uint16_t) ((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return _desc_str;
}
```

---

### 3.6 `PYRo/Peripheral/USB/pyro_usb_cdc_drv.h`（新增，完整）

```cpp
#ifndef __PYRO_USB_CDC_DRV_H__
#define __PYRO_USB_CDC_DRV_H__

#include "pyro_serial_itf.h"
#include "pyro_frame_parser.h"
#include "pyro_task.h"
#include <vector>

namespace pyro
{

/**
 * @brief 基于 TinyUSB(CDC-ACM) 的虚拟串口驱动。
 *
 * 对调用层完全等价于 uart_drv_t —— 本类内部消化所有 USB 与 UART 的差异：
 *  - 链路开启用 start()；接收开关用 enable_rx()/disable_rx()
 *  - 接收在 USB 任务上下文回调，woken 恒为 pdFALSE
 *  - set_frame_config() 生效后，一次回调恰好给到一整帧（内部 frame_parser_t 组帧）
 *
 * @warning instance() 的静态局部变量初始化不是 ISR-safe：必须先由任务上下文调用
 *          start()/instance()，之后 tud_* 回调才会使用它（见 §8.3）。
 * @warning write() 只允许在任务上下文调用，禁止在 ISR 中调用（见 §8.2）。
 */
class usb_cdc_drv_t final : public serial_itf_t
{
  public:
    static usb_cdc_drv_t &instance();

    /** @brief 启动 USB 设备栈（创建内部任务：tusb_init + tud_task 循环）。 */
    status_t start();

    /* ------------------ USB 专有接收控制 ------------------ */
    status_t enable_rx();                     // 等价于 uart_drv_t::enable_rx_dma()
    status_t disable_rx();

    /* ------------------ serial_itf_t 实现 ------------------ */
    status_t write(const uint8_t *p, uint16_t size) override;
    status_t write(const uint8_t *p, uint16_t size, uint32_t waittime) override;
    void     add_rx_event_callback(const rx_event_func &func, uint32_t owner) override;
    status_t remove_rx_event_callback(uint32_t owner) override;
    void     set_frame_config(uint8_t sof, uint16_t frame_len) override;

    /* ------------------ 状态查询 ------------------ */
    [[nodiscard]] bool is_mounted() const;    // 已枚举并配置
    // 注意：write() 当前不依赖 DTR（放宽策略）。本接口供阶段 3 切换到"必须置 DTR"策略时使用，
    //       两种策略的取舍与源码依据见 §8.1。
    [[nodiscard]] bool is_connected() const;  // 已挂载且主机置了 DTR

    /* ------------------ 供 C 回调桥接 ------------------ */
    void on_cdc_rx();                        // tud_cdc_rx_cb -> 读 FIFO + 分发/回环
    void on_mount();                         // tud_mount_cb
    void on_umount();                        // tud_umount_cb
    void on_line_state(bool dtr, bool rts);  // tud_cdc_line_state_cb

  private:
    usb_cdc_drv_t();
    ~usb_cdc_drv_t() override;

    /** @brief 内部 FreeRTOS 任务：先做硬件初始化，再跑 USB 栈。 */
    class usb_task_t final : public task_base_t
    {
      public:
        explicit usb_task_t(usb_cdc_drv_t *owner)
            : task_base_t("usb_task", 256, 512, priority_t::ABOVE_NORMAL),
              _owner(owner) {}

      protected:
        status_t init() override;     // 时钟 / PHY / GPIO / NVIC
        void run_loop() override;     // tusb_init() + while(1) tud_task()

      private:
        usb_cdc_drv_t *_owner;
    };

    struct rx_cb_entry_t
    {
        uint32_t owner;
        rx_event_func func;
    };

    /** @brief 把一段字节流分发给已注册回调（按需组帧）。 */
    void dispatch(const uint8_t *p, uint16_t size);

    /** @brief 回环自测：把收到的字节原样发回。 */
    static void loopback_write(const uint8_t *p, uint16_t size);

    usb_task_t *_task{nullptr};
    std::vector<rx_cb_entry_t> _rx_cbs{};
    frame_parser_t _parser{};
    bool _rx_enabled{false};
};

} // namespace pyro

#endif // __PYRO_USB_CDC_DRV_H__
```

---

### 3.7 `PYRo/Peripheral/USB/pyro_usb_cdc_drv.cpp`（新增，完整）

```cpp
#include "pyro_usb_cdc_drv.h"

#include "tusb.h"
#include "stm32h7xx_hal.h"

// 阶段 1-2：回环自测档位（fail-safe：默认 0 = 关闭）
//   0 = 正常业务路径（交给 dispatch）
//   1 = 字节回环（只验证 USB 收发通路）
//   2 = 经驱动层组帧后再回环（额外验证 frame_parser_t + dispatch，见 §8.5）
// 由 CMake 的 USB_CDC_LOOPBACK 显式设置（见 3.9.1）；阶段 3 接入自瞄时应删除该代码路径。
#ifndef USB_CDC_LOOPBACK
#define USB_CDC_LOOPBACK 0
#endif

namespace pyro
{

/* ============================ Hardware Init ============================ */

status_t usb_cdc_drv_t::usb_task_t::init()
{
    // 说明：USB 的时钟源（HSI48 / PLL1Q / PLL3Q）、USB 电压检测器、OTG_HS 外设时钟、
    //       以及 NVIC 优先级与使能，全部由 CubeMX 生成并维护的 HAL_PCD_MspInit() 完成
    //       （在 CubeMX/Core/Src/usb_otg.c 的 USER CODE 区被显式调用，见 3.8.1b）。
    //       这样时钟树变化（例如把 USB 时钟从 PLL1Q 换成 HSI48）时无需改本驱动。
    //       ⚠️ 切勿在此硬编码 RCC_USBCLKSOURCE_*：与 .ioc 不符会直接导致 USB 无法枚举。

    // DP/DM 引脚：CubeMX 的 gpio.c 未配置 USB 引脚，故在此配置。
    // 内置 FS PHY 固定使用 PA11(D-) / PA12(D+)，AF10。
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {};
    gpio.Pin       = GPIO_PIN_11 | GPIO_PIN_12;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF10_OTG1_HS;   // H723: == GPIO_AF10_OTG2_HS (0x0A)
    HAL_GPIO_Init(GPIOA, &gpio);

    // 强制中断优先级满足 FreeRTOS 约束（必须 <= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY）。
    // MspInit 里已设过一次（值为 5），这里再显式覆盖一次，以防后续 CubeMX 改动优先级，见 §8.6。
    HAL_NVIC_SetPriority(OTG_HS_IRQn,
                         configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);

    return PYRO_OK;
}

void usb_cdc_drv_t::usb_task_t::run_loop()
{
    // 必须在调度器启动后调用（ISR 中使用了 RTOS 队列 API）
    const tusb_rhport_init_t dev_init = {
        .role  = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_FULL,     // 内置 FS PHY
    };
    tusb_init(BOARD_TUD_RHPORT, &dev_init);

    while (true)
    {
        tud_task();                   // 事件处理；期间会回调 tud_cdc_rx_cb()
    }
}

/* ============================ Life Cycle =============================== */

usb_cdc_drv_t::usb_cdc_drv_t() = default;

usb_cdc_drv_t::~usb_cdc_drv_t()
{
    if (_task)
    {
        _task->stop();
        delete _task;
        _task = nullptr;
    }
}

usb_cdc_drv_t &usb_cdc_drv_t::instance()
{
    static usb_cdc_drv_t inst;
    return inst;
}

status_t usb_cdc_drv_t::start()
{
    if (_task)
        return PYRO_ALREADY_INIT;

    _task = new usb_task_t(this);
    if (!_task)
        return PYRO_NO_MEMORY;

    return _task->start();
}

/* ======================= serial_itf_t 实现 ============================= */

status_t usb_cdc_drv_t::write(const uint8_t *p, uint16_t size)
{
    if (!p || size == 0 || !tud_mounted())
        return PYRO_ERROR;

    // 注意：不严格依赖 DTR（部分上位机不置 DTR），可发与否由调用层超时逻辑兜底；
    //       因此"已枚举但无人打开端口"时数据会被静默丢弃（见 §8.1）。若需更强语义，
    //       可改为依赖 tud_cdc_connected()，并把 CFG_TUD_CDC_TX_OVERWRITABLE_IF_NOT_CONNECTED 置 0。
    if (tud_cdc_write_available() < size)
        return PYRO_BUSY;

    const uint32_t written = tud_cdc_write(p, size);
    tud_cdc_write_flush();
    return (written == size) ? PYRO_OK : PYRO_ERROR;
}

status_t usb_cdc_drv_t::write(const uint8_t *p, uint16_t size, uint32_t)
{
    return write(p, size);
}

/* ===================== USB 专有接收控制 ========================== */

status_t usb_cdc_drv_t::enable_rx()
{
#if USB_CDC_LOOPBACK == 2
    // 自测档位 2：用自瞄的帧参数开启组帧，从而验证 frame_parser_t 的切帧行为
    _parser.configure(0xA5, 29);
#endif
    _rx_enabled = true;
    return PYRO_OK;
}

status_t usb_cdc_drv_t::disable_rx()
{
    _rx_enabled = false;
    return PYRO_OK;
}

void usb_cdc_drv_t::add_rx_event_callback(const rx_event_func &func,
                                           uint32_t owner)
{
    rx_cb_entry_t e = {};
    e.owner = owner;
    e.func  = func;
    _rx_cbs.push_back(e);
}

status_t usb_cdc_drv_t::remove_rx_event_callback(uint32_t owner)
{
    for (auto it = _rx_cbs.begin(); it != _rx_cbs.end(); ++it)
    {
        if (it->owner == owner)
        {
            _rx_cbs.erase(it);
            return PYRO_OK;
        }
    }
    return PYRO_NOT_FOUND;
}

void usb_cdc_drv_t::set_frame_config(uint8_t sof, uint16_t frame_len)
{
    _parser.configure(sof, frame_len);
}

bool usb_cdc_drv_t::is_mounted() const
{
    return tud_mounted();
}

bool usb_cdc_drv_t::is_connected() const
{
    return tud_mounted() && tud_cdc_connected();
}

/* ============================ RX / 分发 =============================== */

void usb_cdc_drv_t::dispatch(const uint8_t *p, uint16_t size)
{
    if (!_rx_enabled)
        return;

    BaseType_t woken = pdFALSE;   // 任务上下文：无需 portYIELD

    if (!_parser.enabled())
    {
#if USB_CDC_LOOPBACK == 2
        if (_rx_cbs.empty())   // 自测档位 2：无接收方时回环，验证通路
        {
            loopback_write(p, size);
            return;
        }
#endif
        for (auto &e : _rx_cbs)
        {
            if (e.func(const_cast<uint8_t *>(p), size, woken))
                break;
        }
        return;
    }

    _parser.feed(p, size,
                 [&](const uint8_t *frame, uint16_t len) -> bool
                 {
#if USB_CDC_LOOPBACK == 2
                     if (_rx_cbs.empty())   // 自测档位 2：回环"整帧"，验证组帧结果
                     {
                         loopback_write(frame, len);
                         return true;
                     }
#endif
                     for (auto &e : _rx_cbs)
                     {
                         if (e.func(const_cast<uint8_t *>(frame), len, woken))
                             return true;
                     }
                     return true;   // 继续切下一帧
                 });
}

void usb_cdc_drv_t::loopback_write(const uint8_t *p, uint16_t size)
{
    const uint32_t avail = tud_cdc_write_available();
    const uint32_t n = (size < avail) ? size : avail;
    if (n == 0)
        return;
    tud_cdc_write(p, n);
    tud_cdc_write_flush();
}

void usb_cdc_drv_t::on_cdc_rx()
{
    uint8_t buf[64];

    while (tud_cdc_available())
    {
        const uint32_t n = tud_cdc_read(buf, sizeof(buf));
        if (n == 0)
            break;

#if USB_CDC_LOOPBACK == 1
        // 档位 1：纯字节回环，只验证 USB 收发通路
        loopback_write(buf, (uint16_t) n);
#else
        // 档位 0（正常业务路径）与档位 2（经组帧后回环，见 dispatch 内部 #if）
        dispatch(buf, (uint16_t) n);
#endif
    }
}

void usb_cdc_drv_t::on_mount()
{
    static const char msg[] = "[USB] mounted\r\n";
    loopback_write(reinterpret_cast<const uint8_t *>(msg),
                   (uint16_t) (sizeof(msg) - 1));
}

void usb_cdc_drv_t::on_umount()
{
    _parser.reset();              // 丢弃半帧，重新同步
    tud_cdc_read_flush();
}

void usb_cdc_drv_t::on_line_state(bool dtr, bool rts)
{
    (void) rts;
    // 用 sizeof 而非硬编码长度，避免字符串变更时静默截断/越界
    static const char dtr_on[]  = "[USB] dtr=1\r\n";
    static const char dtr_off[] = "[USB] dtr=0\r\n";
    const char *msg = dtr ? dtr_on : dtr_off;
    loopback_write(reinterpret_cast<const uint8_t *>(msg),
                   (uint16_t) (dtr ? (sizeof(dtr_on) - 1)
                                   : (sizeof(dtr_off) - 1)));
}

} // namespace pyro

/* ============================ C 桥接层 ================================ */
extern "C" {

void tud_cdc_rx_cb(uint8_t itf)
{
    (void) itf;
    pyro::usb_cdc_drv_t::instance().on_cdc_rx();
}

void tud_mount_cb(void)
{
    pyro::usb_cdc_drv_t::instance().on_mount();
}

void tud_umount_cb(void)
{
    pyro::usb_cdc_drv_t::instance().on_umount();
}

void tud_suspend_cb(bool remote_wakeup_en)
{
    (void) remote_wakeup_en;
}

void tud_resume_cb(void)
{
}

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
    (void) itf;
    pyro::usb_cdc_drv_t::instance().on_line_state(dtr, rts);
}

/**
 * @brief 供 CubeMX 生成的 OTG_HS_IRQHandler 转发的桥接函数。
 * @note 中断向量定义保留在 CubeMX/Core/Src/stm32h7xx_it.c（其 USER CODE 区调用本函数），
 *       这样 CubeMX 重新生成代码不会造成符号丢失或重复（见 3.8.2）。
 *       编号必须与 tusb_init()/BOARD_TUD_RHPORT 一致。
 */
void pyro_usb_irq_handler(void)
{
    tusb_int_handler(BOARD_TUD_RHPORT, true);
}

} // extern "C"
```

---

### 3.8 CubeMX 侧修改（3 处，全部位于 USER CODE 区 —— 重新生成代码不会丢失）

> ⚠️ 原方案（注释 `main.c` 的调用 + 删除 `it.c` 的 IRQ handler）**已被废弃**：CubeMX 重新生成会把它们复原，
> 且会造成 `OTG_HS_IRQHandler` 符号重复（链接失败）。最终实现改为下述 USER CODE 方案。

#### 3.8.1 `CubeMX/Core/Src/usb_otg.c` —— 禁止 HAL PCD 初始化

在 `USER CODE BEGIN 0` 内加一行宏定义，把 `HAL_PCD_Init()` 变成空操作：
调用点保留（**`main.c` 无需改动**），该调用只填结构体、不再接触任何 USB 寄存器。

```diff
 /* USER CODE BEGIN 0 */
+/* [PYRo] 禁止 HAL PCD 初始化：USB 由 TinyUSB(dwc2) 独占 OTG_HS 核心。
+ *        把 HAL_PCD_Init 宏化为空操作后，本文件下方的 MX_USB_OTG_HS_PCD_Init()
+ *        只填结构体、不再接触任何 USB 寄存器，因此 main.c 无需修改。
+ *        时钟/PHY/GPIO/NVIC 初始化见 PYRo/Peripheral/USB/pyro_usb_cdc_drv.cpp。
+ *        本改动位于 USER CODE 区，CubeMX 重新生成代码不会丢失。 */
+#define HAL_PCD_Init(hpcd) (HAL_OK)
 /* USER CODE END 0 */
```

#### 3.8.1b `CubeMX/Core/Src/usb_otg.c` —— 显式执行 MspInit（保住 CubeMX 维护的 USB 时钟）

`HAL_PCD_Init()` 被宏化为空操作后，HAL 不会再调用 `HAL_PCD_MspInit()`，因此必须显式调用一次，
以保留 CubeMX 维护的 USB 时钟源 / 电压检测 / 时钟使能 / NVIC：

```diff
   /* USER CODE BEGIN USB_OTG_HS_Init 2 */
+  /* [PYRo] HAL_PCD_Init 已被宏化为空操作（见 USER CODE BEGIN 0），
+   *        这里显式调用 MspInit，只保留 CubeMX 维护的那部分初始化：
+   *          - USB 时钟源（当前 = RCC_USBCLKSOURCE_HSI48，随时钟树自动更新）
+   *          - USB 电压检测器使能
+   *          - OTG_HS 外设时钟使能
+   *          - OTG_HS 中断优先级与使能 */
+  HAL_PCD_MspInit(&hpcd_USB_OTG_HS);
   /* USER CODE END USB_OTG_HS_Init 2 */
```

> ⚠️ **关键约束：不要在 USB 驱动里硬编码 `RCC_USBCLKSOURCE_*`。**
> 本项目 USB 时钟为 **HSI48**（`usb_otg.c:80`、`main.c:188-190`）；
> 若驱动里擅自改配 `RCC_USBCLKSOURCE_PLL`（本例 PLL1Q ≈ 125MHz），USB 将完全无法枚举。

#### 3.8.2 `CubeMX/Core/Src/stm32h7xx_it.c` —— 中断转发（保留向量定义）

**中断向量 `OTG_HS_IRQHandler` 保留由 CubeMX 生成**（避免符号丢失/重复），只在两个 USER CODE 区插入内容：

```diff
 /* USER CODE BEGIN Includes */
+/* [PYRo] USB 中断转发桥接函数，定义在 PYRo/Peripheral/USB/pyro_usb_cdc_drv.cpp。
+ *        此处仅做声明，避免在本文件包含 tusb.h（那需要 TinyUSB 的宏环境）。 */
+extern void pyro_usb_irq_handler(void);
 /* USER CODE END Includes */
```

```diff
 void OTG_HS_IRQHandler(void)
 {
   /* USER CODE BEGIN OTG_HS_IRQn 0 */
-
+  /* [PYRo] USB 中断改由 TinyUSB 处理：转发后立即返回，不再执行下面的 HAL_PCD_IRQHandler。
+   *        中断向量保留由 CubeMX 生成，故重新生成代码不会造成符号丢失或重复。 */
+  pyro_usb_irq_handler();
+  return;
   /* USER CODE END OTG_HS_IRQn 0 */
   HAL_PCD_IRQHandler(&hpcd_USB_OTG_HS);      /* 永不执行（死代码，保留以免动用生成区） */
   /* USER CODE BEGIN OTG_HS_IRQn 1 */
-
   /* USER CODE END OTG_HS_IRQn 1 */
 }
```

#### 3.8.3 `CubeMX/Core/Inc/stm32h7xx_it.h`

**不动**：`void OTG_HS_IRQHandler(void);` 声明与 CubeMX 生成的向量定义匹配，保持原样。

---

### 3.9 构建与源文件接入

#### 3.9.1 顶层 `CMakeLists.txt`（插入在 `add_subdirectory(PYRo)` 之前，即第 65 行前）

```cmake
# ==============================================================================
# 4.5 TinyUSB（虚拟串口）
# ==============================================================================
set(TINYUSB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/TinyUSB")

add_library(tinyusb STATIC)
# 注意：此处必须用 include() 而非 add_subdirectory()：官方 src/CMakeLists.txt 只定义函数
#       （tinyusb_target_add / tinyusb_sources_get），而 CMake 的函数定义不跨目录作用域传播，
#       用 add_subdirectory 会导致后续调用报 "Unknown CMake command tinyusb_target_add"。
include(${TINYUSB_DIR}/src/CMakeLists.txt)
tinyusb_target_add(tinyusb)                  # common + device + class（host/typec 由宏关闭为空编译）

# DCD（设备控制器驱动）不在 tinyusb_target_add 内，必须显式加入
target_sources(tinyusb PRIVATE
        ${TINYUSB_DIR}/src/portable/synopsys/dwc2/dcd_dwc2.c
        ${TINYUSB_DIR}/src/portable/synopsys/dwc2/dwc2_common.c)

target_include_directories(tinyusb PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/PYRo/Peripheral/USB)   # tusb_config.h 所在处

target_compile_definitions(tinyusb PUBLIC
        CFG_TUSB_MCU=OPT_MCU_STM32H7
        CFG_TUSB_OS=OPT_OS_FREERTOS
        CFG_TUSB_DEBUG=0)

target_link_libraries(tinyusb PUBLIC stm32cubemx)          # 传递 FreeRTOS/CMSIS/HAL 头文件

# ---- 阶段 1-2 临时开关（阶段 3 接入自瞄时必须删除本段）----
# USB_CDC_LOOPBACK: 0=正常业务路径 1=字节回环 2=经组帧后回环（见 3.7 与 §8.5）
# 默认取档位 2，可在阶段 1-2 顺带验证 frame_parser_t 的切帧行为；
# 注意源码内默认值是 0（fail-safe），只有本开关显式打开时才进入回环路径。
option(USB_CDC_SELF_TEST "USB CDC 回环自测（阶段1-2）" ON)
if (USB_CDC_SELF_TEST)
    add_compile_definitions(USB_CDC_LOOPBACK=2)
    add_compile_definitions(USB_CDC_STANDALONE_TEST=1)
else()
    add_compile_definitions(USB_CDC_LOOPBACK=0)
endif()
```

#### 3.9.2 `PYRo/CMakeLists.txt`

```diff
 target_sources(PYRo PRIVATE
         ...
         Peripheral/UART/pyro_uart_drv.cpp
         Peripheral/UART/pyro_bsp_uart.cpp
+        Peripheral/USB/pyro_usb_cdc_drv.cpp
+        Peripheral/USB/pyro_usb_descriptors.c
         ...
 )
@@
         # --- Peripheral ---
         Peripheral/CAN
         Peripheral/DWT
         Peripheral/UART
+        Peripheral/Serial
+        Peripheral/USB
@@
-target_link_libraries(PYRo PUBLIC stm32cubemx dsppp_lib)
+target_link_libraries(PYRo PUBLIC stm32cubemx dsppp_lib tinyusb)
```

#### 3.9.3 `Robot/Infantry2/pyro_init_thread.cpp`（临时，仅阶段 1-2）

```diff
 #include "pyro_supercap_drv.h"
+#ifdef USB_CDC_STANDALONE_TEST
+#include "pyro_usb_cdc_drv.h"
+#endif
@@
 #ifdef AUTOAIM_UART
         AUTOAIM_UART.reset(921600, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                            UART_PARITY_NONE);
         AUTOAIM_UART.enable_rx_dma();
 #endif
+
+#ifdef USB_CDC_STANDALONE_TEST
+        /* [阶段1-2 临时] 单独启动 USB CDC 做枚举/回环/DTR 验证，不接自瞄 */
+        usb_cdc_drv_t::instance().start();
+#endif
 
         vTaskDelete(nullptr);
```

---

### 3.10 验收标准

> 说明：构建验证以本方案新增/修改的编译单元为单位。若仓库中存在既有问题导致整体链接失败，按其既有状态处理，**本方案不做修改**。

| # | 检查项 | 通过标准 |
|---|---|---|
| 1 | 编译 | 链接通过；在 map 中搜索 `cdc_device`、`dcd_dwc2`、`pyro_usb_cdc_drv` 确认已被链接（静态库形式通常显示为 `libtinyusb.a(cdc_device.c.obj)`） |
| 2 | 枚举 | 设备管理器出现「USB 串行设备 (COMx)」，属性可见 `USB\VID_CAFE&PID_4010` |
| 3 | 挂载 | 打开端口收到 `[USB] mounted` |
| 4 | DTR | 打开端口收到 `[USB] dtr=1`（关闭时 `[USB] dtr=0`） |
| 5 | 回环 | 档位 1：发送任意数据原样回显；档位 2：连续发送 3 帧自瞄格式数据，按帧原样返回（验证 `frame_parser_t` 切帧正确） |
| 6 | 引脚确认 | PA12(D+) 有 1.5k 上拉、总线存在 1ms 帧活动（示波器/逻辑分析仪） |
| 7 | 稳定性 | 连续插拔 10 次，每次均可重新枚举并回环；无死机/HardFault |
| 8 | 零回归 | DR16 拨杆/摇杆动作生效；VT03 数据刷新；裁判系统 UI 正常；UART7 自瞄 `check_online()` 仍为真；CAN 电机正常 |
| 9 | 吞吐（可选） | 连续灌 1MB 数据回环，无持续丢包 |

**第 4/6 项是本阶段的核心目的**：DTR 结果决定阶段 3 的 TX 策略；引脚结果决定方案是否需要改 PHY 配置。

### 3.11 失败排查表

| 症状 | 定位方向 |
|---|---|
| PC 完全无反应 | ① `CFG_TUD_MAX_SPEED` 非 FULL；② PA11/PA12 未配置或硬件引脚不符；③ `MX_USB_OTG_HS_PCD_Init()` 未注释；④ `OTG_HS_IRQHandler` 仍指向 HAL |
| 枚举后立即掉线 | `tusb_int_handler` 的编号与 `BOARD_TUD_RHPORT` 不一致 |
| 能枚举但无 `mounted` | USB 任务未启动 / 任务栈不足 / `start()` 未被调用 |
| 无 `dtr=1` | 上位机不置 DTR（记录即可；`write()` 已不严格依赖 DTR） |
| 回显丢字节 | 回环未检查 `tud_cdc_write_available()`（本方案已检查） |
| HardFault | 中断优先级 > 5（FreeRTOS `FromISR` 断言）；或任务栈过小（512 → 768 词） |
| 链接期符号重复 | `stm32h7xx_it.c` 中的 `OTG_HS_IRQHandler` 未删除 |

### 3.12 回滚

- **最小回滚**：CMake 中 `USB_CDC_SELF_TEST=OFF` → USB 不再启动，设备恢复原状；
- **完全回滚**：还原 `main.c`、`stm32h7xx_it.c/.h` 三处改动，移除 `tinyusb` 链接与新增目录；
- 本阶段未触碰任何现有链路代码，**不存在"USB 改动破坏原有 UART 功能"的可能性**。

---

## 4. 阶段 3：接入自瞄（概要，待阶段 1-2 通过后展开）

| 改动点 | 文件 | 内容 |
|---|---|---|
| 接口落地 | `PYRo/Peripheral/UART/pyro_uart_drv.h` | `class uart_drv_t : public serial_itf_t`；`write×2 / add_rx_event_callback / remove_rx_event_callback` 加 `override`；`set_frame_config` 实现为 no-op。**`.cpp` 不需修改**；全仓无任何类继承 `uart_drv_t`，兼容性已核查 |
| 接口瘦身 | `PYRo/Peripheral/Serial/pyro_serial_itf.h` | 删除 `reset / enable_rx_dma / disable_rx_dma` 三个纯虚——三者均不被 `serial_itf_t*` 调用（判据：**谁调用**，而非"谁 no-op"）。接口只保留消费者真正经指针调用的四个：`write×2 / add_rx_event_callback / remove_rx_event_callback / set_frame_config`；`uart_drv_t::reset / enable_rx_dma / disable_rx_dma` 去 `override` 变非虚，实现与语义零变化 |
| USB 专有 API | `PYRo/Peripheral/USB/pyro_usb_cdc_drv.h/.cpp` | 删除 `reset()` 空实现；`enable_rx_dma()/disable_rx_dma()` 改名 `enable_rx()/disable_rx()`（USB 无 DMA，去实现细节语义词） |
| 依赖抽象 | `Robot/Infantry2/Communication/Gimbal_board/pyro_autoaim_drv.h/.cpp` | `uart_drv_t *_uart_drv` → `serial_itf_t *_serial_itf`；构造函数参数同步；`get_instance()` 增加 `#elif defined(AUTOAIM_USB_CDC)` 分支返回 `&usb_cdc_drv_t::instance()` |
| 帧配置 | `pyro_autoaim_drv.cpp::init_impl()` | 新增一行 `_serial_itf->set_frame_config(FRAME_SOF, sizeof(rx_packet_t));`（驱动层组帧，`rx_callback` 逻辑不变） |
| 切换宏 | `Robot/Infantry2/CMakeLists.txt` | `AUTOAIM_UART=PYRO_UART7` 注释保留 → `AUTOAIM_USB_CDC=1` |
| 初始化 | `Robot/Infantry2/pyro_init_thread.cpp` | USB 分支执行 `usb_cdc_drv_t::instance().start()` + `enable_rx()`（USB 无链路参数，故无 `reset` 这一步） |
| 清理测试开关 | `CMakeLists.txt` + `pyro_usb_cdc_drv.cpp` | 删除 `USB_CDC_SELF_TEST` 开关段与回环代码路径（`USB_CDC_LOOPBACK` 相关 `#if`），避免误留劫持业务数据（见 A2 说明） |

**承诺**：`pyro_autoaim_com.cpp` 永不修改；UART/USB 通过宏一键回退，回退成本 = 改一行 + 重编译。

---

## 5. 风险清单

| # | 风险 | 严重度 | 对策 |
|---|---|---|---|
| ① | 速度宏误配为 HS/默认 → `dwc2_phy_init()` 清 `GCCFG.PWRDWN` 关闭内置 FS PHY → 完全不枚举 | 致命 | 锁死 `CFG_TUD_MAX_SPEED = OPT_MODE_FULL_SPEED`，并在 `tusb_config.h` 注释中交叉引用 |
| ② | RHPORT 编号与 ISR 中 `tusb_int_handler(n)` 不一致 | 致命 | 统一使用 1（`BOARD_TUD_RHPORT`），两处引用同一宏 |
| ③ | 中断优先级必须 ≤ `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`(=5) | 高 | `usb_task_t::init()` 中显式设置 |
| ④ | CubeMX 重生成覆盖 `main.c` / `stm32h7xx_it.c` | 中 | 全部以注释标注；USB 初始化集中在 `usb_cdc_drv_t` 内 |
| ⑤ | 硬件 D+/D- 可能不在 PA11/PA12（若走 ULPI 外部 PHY 则方案需调整） | 高 | 阶段 1-2 用回环 + 引脚确认验证；不通过则回到硬件确认 |
| ⑥ | USB 为纯字节流，会粘包/拆包（UART IDLE 语义不成立） | 高 | 驱动层 `frame_parser_t` 组帧，调用层无感 |
| ⑦ | 回调上下文由 ISR 变为任务 | 中 | 传 `woken = pdFALSE`；回调内禁止 `portYIELD_FROM_ISR`、禁止耗时/阻塞操作 |
| ⑧ | 拔插残留半帧 | 中 | `tud_umount_cb` 内 `_parser.reset()` + `tud_cdc_read_flush()` |
| ⑨ | 与 HAL_PCD 争用同一 OTG_HS 核心 | 高 | 停用 `MX_USB_OTG_HS_PCD_Init()`；IRQ 迁移到 TinyUSB |
| ⑩ | TinyUSB 全部 `.c` 需宏一致，否则结构体布局不一致 | 中 | 宏用 `target_compile_definitions(... PUBLIC)`，由 `tinyusb` 目标向 `PYRo` 传播 |
| ⑪ | USB 线缆/整机电磁干扰导致枚举不稳定（机器人上常见） | 中 | 使用屏蔽线、缩短走线、远离电机驱动线；必要时加共模磁环/ESD 器件。枚举失败时优先排查线缆 |
| ⑫ | PC 休眠 / SUSPEND 期间自瞄链路表现为掉线 | 低 | 由调用层 50ms 无收包超时自然处理；`tud_resume_cb` 触发后链路自动恢复，无需特殊处理 |

---

## 6. 待确认项（需硬件/上位机侧信息）

1. **USB 物理引脚**：是否为 PA11(D-) / PA12(D+)（内置 FS PHY 的唯一选择）；若板子实际走 ULPI，则需改 PHY 与 AF 配置。
2. **描述符 VID/PID**：当前占位 `0xCAFE:0x4010`，需确认正式值（影响是否需要 `.inf`，通常 CDC 免驱）。
3. **上位机是否置 DTR**：决定阶段 3 的 TX 策略（本方案 `write()` 已放宽为只检查 `tud_mounted()`）。
4. **上位机是否按固定 COM 号打开**：USB CDC 的 COM 号会在插拔/换口时变化，若写死需改为按 VID/PID 或端口描述匹配。
5. **USB 时钟精度（HSI48 + CRS）**：当前 USB 时钟为 HSI48 且**未启用 CRS**；若出现枚举或连接不稳定，按 §8.11 处理。

---

## 7. 不在本方案范围内

- `bugs.md` 中记录的任何既有问题（如 §1 `referee_msg` 静态/extern 链接冲突、§2 底盘板编译、§5 自瞄状态未接线、§8 `vPortDmaFree` 等）**一律不做修改**；本方案不依赖、也不触发对它们的修复。若这些既有问题导致整体构建失败，按其既有状态处理。
- 调试串口（VOFA / JCOM）改造、第二路 CDC、HS/ULPI 支持、DMA 模式（`CFG_TUD_DWC2_DMA_ENABLE=1`，需 32 字节对齐 + dcache 维护）。

---

## 8. 补充设计约束与决策记录（A/B 类改进）

> 本节汇总 §3 之外的设计约束、源码依据与取舍决策。对应的代码/配置修正已就地体现在 §3.2 / §3.4 / §3.6 / §3.7 / §3.9 / §3.10 / §5。

### 8.0 改进项索引（A/B 类 15 项落地位置）

| 项 | 主题 | 落地位置 |
|---|---|---|
| A1 | `on_line_state` 硬编码长度 13 | §3.7 `on_line_state()` 改用 `sizeof(dtr_on/_off) - 1` |
| A2 | 回环默认值 fail-safe | §3.7 源码默认 `0`；§3.9.1 CMake 显式置 `2`；§4 "清理测试开关" |
| A3 | 验收第 1 项措辞（静态库 map 形式） | §3.10 第 1 项 |
| A4 | `is_connected()` 无调用点 | §3.6 注释指向 §8.1；§8.1 给出切换决策流程 |
| B1 | DTR 策略与 TX 覆盖行为 | §3.4 宏与注释 + §3.7 `write()` 注释 + §8.1 |
| B2 | TX 侧上下文约束 | §3.2 `write()` 的 `@note` + §8.2 |
| B3 | 单例构造时序 | §3.6 类注释 `@warning` + §8.3 |
| B4 | 回调缓冲生命周期 | §3.2 `rx_event_func` 的 `@warning` + §8.4 |
| B5 | 阶段 1-2 组帧验收覆盖 | §3.7 档位 2 分支 + §3.9.1 + §3.10 第 5 项 + §8.5 |
| B6 | 中断使能归属 | §3.7 `usb_task_t::init()` + §8.6 |
| B7 | `BOARD_TUD_MAX_SPEED` 冗余性 | §3.4 注释 + §8.7 |
| B8 | CubeMX 重生成复查清单 | §8.8 |
| B9 | `include()` 而非 `add_subdirectory()` 的原因 | §3.9.1 注释 |
| B10 | 零回归的具体观察点 | §3.10 第 8 项 + §8.9 |
| B11 | 新增风险条目 ⑪⑫ | §5 风险表 + §8.10 |
| — | **USB 时钟源硬编码错误（本项目为 HSI48）** | §3.7 `init()` 移除硬编码 + §3.8.1b 显式调用 MspInit + §8.11 |
| — | **FreeRTOS V10.3.1 缺 `pdTICKS_TO_MS`（编译报错）** | `tusb_config.h` 内补 `#define`（带 `#ifndef` 守卫）+ §8.12 |

### 8.1 DTR 策略与 TX 覆盖行为（B1）

**源码依据**：`src/class/cdc/cdc_device.c:395-415`，主机 `SET_CONTROL_LINE_STATE` 时按 DTR 设置 FIFO 覆盖属性：

```c
#if CFG_TUD_CDC_TX_OVERWRITABLE_IF_NOT_CONNECTED
    const bool is_overwritable = !dtr;
#else
    const bool is_overwritable = false;
#endif
    tu_fifo_set_overwritable(&p_cdc->tx_stream.ff, is_overwritable);
```

**后果**：默认配置（本方案取 1）下，当上位机未打开端口（DTR=0）时 TX FIFO 处于"可覆盖"状态：

- `tud_cdc_write()` 不会因缓冲满而失败，`tud_cdc_write_available()` 也不反映"不可写"；
- 因此 `usb_cdc_drv_t::write()` 通常返回 `PYRO_OK`，但**数据被静默丢弃**；
- 调用层 `check_online()` 只能靠 50ms 无收包超时感知"无人接收"，无法从发送结果感知。

**两种可选策略**：

| 策略 | 配置 | `write()` 判断 | 优点 | 缺点 |
|---|---|---|---|---|
| **S1（当前）** | `CFG_TUD_CDC_TX_OVERWRITABLE_IF_NOT_CONNECTED = 1` | 仅 `tud_mounted()` | 兼容"不置 DTR 的上位机"；发送极少失败，行为更接近 UART | "无人接收"不可从发送侧观测 |
| **S2（可选切换）** | `... = 0` + 依赖 `tud_cdc_connected()` | 未连接直接 `PYRO_ERROR` | 发送失败可被上层立刻观测，`check_online()` 更灵敏 | 要求上位机必须置 DTR，否则永远发不出去 |

**决策流程**：由阶段 1-2 验收第 4 项（能否收到 `[USB] dtr=1`）决定 ——
- 能收到 → 可切 S2（语义更严格）；
- 收不到 → 必须保持 S1。

`usb_cdc_drv_t::is_connected()` 即为此切换预留的接口。

### 8.2 TX 侧上下文约束（B2）

**源码依据**：`src/common/tusb_fifo.h:23` 定义 `#define CFG_FIFO_MUTEX OSAL_MUTEX_REQUIRED`；非 `OPT_OS_NONE` 时 `OSAL_MUTEX_REQUIRED = 1`（`src/osal/osal.h:27-31`）；`src/common/tusb_fifo.c:17` 的 `ff_lock()` 以 `OSAL_TIMEOUT_WAIT_FOREVER` 加锁。

**结论**：
- 跨任务调用 `tud_cdc_write()` / `tud_cdc_write_flush()` 是**安全的**（自瞄任务写入 + `tud_task()` 内部 flush 不会破坏 FIFO 一致性）；
- 但**绝不能在 ISR 中调用**（ISR 里无法获取 mutex），也不要在关中断临界区内调用（会阻塞等待）；
- 该约束已写入 `serial_itf_t::write()` 的 `@note`。

### 8.3 单例构造时序（B3）

`usb_cdc_drv_t::instance()` 使用函数内静态局部变量，其初始化（C++11 起带线程安全 guard）**不是 ISR-safe**。

**约束**：必须先在**任务上下文**调用 `instance()` 或 `start()`；此后 `tud_*` 回调内部再调用 `instance()` 才是安全的。
当前设计中 `pyro_init_thread` 会先调用 `start()`，满足该前提。已在 `usb_cdc_drv_t` 类注释中标注。

### 8.4 回调缓冲生命周期（B4）

`dispatch()` 传给回调的 `frame` 指向 `frame_parser_t::_buf`（驱动内部成员）；`uart_drv_t` 路径同理指向其 DMA 双缓冲。

**约束**：该指针**仅在本次回调期间有效**，回调方必须自行拷贝后才能异步使用。
现有调用层（`infantry2_autoaim_drv_t::rx_callback`）用 `xMessageBufferSendFromISR` 拷贝进 MessageBuffer，满足约束；阶段 3 不得改为"仅保存指针"。

### 8.5 阶段 1-2 的验收覆盖度（B5）

纯字节回环（档位 1）**不会执行 `dispatch()` 与 `frame_parser_t`**，等于把阶段 3 的核心新逻辑留到首次上机才验证。

因此引入档位 2：`enable_rx_dma()` 内以自瞄帧参数（SOF = 0xA5, len = 29）开启组帧，`dispatch()` 在"无注册回调"时把**整帧**回写。
验收观察法：连续发送 3 帧数据，若稳定地按帧原样返回（不多不少），即证明切帧/粘包处理正确。

### 8.6 中断使能归属（B6）

`HAL_NVIC_EnableIRQ()` 与 TinyUSB 内部的 `usbd_init() -> dcd_int_enable()`（`src/device/usbd.c:581`）重复。
本方案只设置优先级、不重复使能；若实测枚举异常，可临时打开该行作对照排查。

### 8.7 `BOARD_TUD_MAX_SPEED` 的冗余性（B7）

`TUD_OPT_HIGH_SPEED` 仅由 `CFG_TUD_MAX_SPEED` 决定（`src/tusb_option.h:502`）；`BOARD_TUD_MAX_SPEED` 在 STM32 dwc2 端口中不参与判断，仅为兼容 `board.mk` 惯例保留。

### 8.8 CubeMX 重新生成后的复查清单（B8）

每次用 CubeMX 重新生成代码后必须复查以下 3 点，否则典型表现为"USB 突然不枚举"：

| # | 检查点 | 期望状态 |
|---|---|---|
| 1 | `CubeMX/Core/Src/usb_otg.c` 的 `USER CODE BEGIN 0` | 含 `#define HAL_PCD_Init(hpcd) (HAL_OK)`（缺失则 HAL 与 TinyUSB 争用 OTG_HS 核心） |
| 2 | `CubeMX/Core/Src/stm32h7xx_it.c` 的 `OTG_HS_IRQHandler` | 保持 CubeMX 生成的原样，且 `USER CODE BEGIN OTG_HS_IRQn 0` 内含 `pyro_usb_irq_handler(); return;`（缺失则中断仍走 HAL） |
| 3 | `CubeMX/Core/Src/gpio.c` 是否重新配置 PA11/PA12 | 不应被其它外设抢占（USB 引脚由 `usb_task_t::init()` 配为 AF10） |

### 8.9 零回归的具体观察点（B10）

已并入 §3.10 验收表第 8 项：DR16 拨杆/摇杆动作、VT03 数据刷新、裁判系统 UI 刷新、UART7 自瞄 `check_online()` 仍为真、CAN 电机正常。

### 8.10 新增风险条目（B11）

已并入 §5 风险清单：⑪ USB 线缆/整机电磁干扰导致枚举不稳定；⑫ PC 休眠 / SUSPEND 期间链路表现为掉线。

### 8.11 USB 时钟源：HSI48 与 CRS（重要）

**现状**：本项目 USB 时钟源为 **HSI48** —— `usb_otg.c:80` 为 `RCC_USBCLKSOURCE_HSI48`，
`main.c:188-190` 使能 `RCC_HSI48_ON`。同时 HSE + PLL1 供 SYSCLK，
本例 PLL1 配置（`PLLM=2, PLLN=40, PLLQ=4`）下 **PLL1Q ≈ 125MHz ≠ 48MHz**，
因此 **USB 绝不能用 `RCC_USBCLKSOURCE_PLL`**。

**设计约束**：USB 驱动层**不得**配置或改动 USB 时钟源；统一由 CubeMX 的 `HAL_PCD_MspInit()`
负责（见 3.8.1b），这样 `.ioc` 时钟树变化时驱动零改动。

**CRS 提醒**：当前工程**未启用 CRS**（无 `MX_CRS_Init` / `crs.c`）。
HSI48 出厂精度（典型 ±1~3%）宽于 USB FS 规范要求的 ±0.25%，
实践中多数情况可正常枚举，但属边缘情形。若出现 **枚举不稳定 / 偶发掉线 / 长时间运行后断连**，
按以下顺序排查：
1. 启用 CRS（Clock Recovery System），同步源选 **USB SOF**（设备模式下从主机 SOF 获得），
   让 HSI48 被持续微调；
2. 或改用 HSE + PLL3Q 产生 48MHz 供 USB（需在 `.ioc` 时钟树中调整）。

### 8.12 FreeRTOS V10.3.1 兼容补丁：`pdTICKS_TO_MS`

**现象**：
```
third_party/TinyUSB/src/osal/osal_freertos.h:97:10: error:
  implicit declaration of function 'pdTICKS_TO_MS' [-Wimplicit-function-declaration]
```

**根因**：`pdTICKS_TO_MS()` 是 **FreeRTOS V10.4.0** 才引入的宏，
而本项目 FreeRTOS 为 **V10.3.1**（`CubeMX/Middlewares/.../include/task.h:46` → `tskKERNEL_VERSION_NUMBER "V10.3.1"`）。
TinyUSB 的 `osal_time_millis()` 依赖它。

**对策**：在 `PYRo/Peripheral/USB/tusb_config.h` 中按 FreeRTOS V11 的语义补上（带 `#ifndef` 守卫）：

```c
#ifndef pdTICKS_TO_MS
#define pdTICKS_TO_MS(xTicks) \
    ((uint32_t)(((uint64_t)(xTicks) * 1000U) / configTICK_RATE_HZ))
#endif
```

可行性依据：包含链为 `osal.h → common/tusb_common.h → tusb_option.h → tusb_config.h`，
故该宏的定义一定早于 `osal_freertos.h:97` 的使用点；
且 `osal_freertos.h` 已在文件头 `#include "FreeRTOS.h"`，使用点处 `configTICK_RATE_HZ` 必然可见。

**兼容性核查（已逐项对照 V10.3.1）**：TinyUSB 的 FreeRTOS OSAL 其余 API 在 V10.3.1 中均存在 ——
`xSemaphoreCreateBinaryStatic` / `xSemaphoreCreateMutexStatic` / `xQueueCreateStatic` /
`xSemaphoreGiveFromISR` / `xQueueSendToBackFromISR` / `xQueueReset` / `uxQueueMessagesWaiting` /
`pdMS_TO_TICKS`（`projdefs.h`）/ `taskENTER_CRITICAL` / `vQueueAddToRegistry`（本项目
`configQUEUE_REGISTRY_SIZE = 8`）。即 **`pdTICKS_TO_MS` 是唯一一处版本不兼容**。

### 8.13 自查发现的其它问题（与 USB 无关，但直接决定"接入自瞄"能否生效）

#### 8.13.1 [P0，已修] `infantry2_autoaim_init()` 从未被调用

**位置**：`Robot/Infantry2/pyro_mission_planer.cpp`（仅在文件顶部 `extern` 声明，`start_mission_planer_task()` 里从未 `xTaskCreate`）。

**后果**（与链路是 UART 还是 USB-CDC 无关，属纯遗漏）：
- `infantry2_autoaim_drv_t::get_instance()` 未被调用 → 驱动实例不存在；
- `start_rx()` 未被调用 → `init_impl()` 未执行 → **`add_rx_event_callback()` 未注册**，且组帧未配置；
- `infantry2_autoaim_app_thread` 未创建 → **不发送遥测、不更新 `autoaim_cmd`**。
- 即：自瞄收发**全程不工作**，表现为"USB 枚举正常但毫无数据"。

**修复**：在 `#if BOARD == GIMBAL_BOARD` 分支补一条 `xTaskCreate`；同时把
`pyro_autoaim_com.cpp` 中 `infantry2_autoaim_init()` 的签名统一为 `(void *argument)`，与实际作为任务入口使用保持一致。

> 注：该项**不在 `bugs.md` 的记录范围内**，属于本次接入过程中新发现的问题。

#### 8.13.2 [P1，已修] `_rx_cbs` 跨任务并发访问

**风险**：`add_rx_event_callback()` / `remove_rx_event_callback()` 由各模块的 init 任务调用（会触发 `std::vector` 的 `push_back`/`erase`），
而 `dispatch()` 在 USB 任务上下文中遍历同一容器。二者优先级相同可被抢占 → **vector 重新分配期间迭代 = 迭代器失效 / 堆破坏**。
（`uart_drv_t` 的同类结构存在相同形态的隐患，属框架既有问题，本方案不改动 UART 侧。）

**修复**：
- `_rx_cbs.reserve(MAX_RX_CALLBACKS)`（构造时预分配，注册路径不再分配内存）；
- `add` / `remove` 用 `taskENTER_CRITICAL()/taskEXIT_CRITICAL()` 保护；
- `dispatch()` 在短临界区内取**固定大小快照**（`rx_event_func cbs[MAX_RX_CALLBACKS]`），**回调在临界区外执行**，避免长时间关中断；
- 新增 `MAX_RX_CALLBACKS = 4` 上限（超出则忽略注册，避免无限增长）。

#### 8.13.3 [P2，已修] 帧长与组帧缓冲容量的隐式耦合

`frame_parser_t::configure()` 在 `frame_len > MAX_FRAME_LEN` 时会**静默把帧长置 0（关闭组帧）**，
此时 USB 路径退化为"按字节流回调"，而调用层的 `size == sizeof(rx_packet_t)` 判断几乎永不成立 → **长期无有效数据且无任何报错**。

**修复**：在 `pyro_autoaim_drv.cpp::init_impl()` 内加编译期断言：

```cpp
static_assert(sizeof(rx_packet_t) <= frame_parser_t::MAX_FRAME_LEN,
              "autoaim rx frame exceeds frame_parser_t::MAX_FRAME_LEN");
```

当前实测：`rx_packet_t = 29B` ≤ `MAX_FRAME_LEN = 64B` ✓。

#### 8.13.4 [P3，建议未改] 描述符 Serial Number 为固定串

当前 `"PYRO-INF2-0001"` 固定不变 → 同一台 PC 上接入多台机器人时，
Windows 会把它们识别为**同一个设备**（VID/PID/Serial 相同），可能出现 COM 口互相覆盖。
建议后续从 STM32 UID（`UID_BASE`）生成序列号字符串，使每块板子唯一。

---

## 附录 A：调用层无感对照表

| 维度 | 普通 UART（`uart_drv_t`） | USB CDC（`usb_cdc_drv_t`） | 调用层可见差异 |
|---|---|---|---|
| 帧格式 / CRC / 结构体 | 不变 | 不变 | **无** |
| `reset(baud, ...)` | 真实生效 | no-op，返回 `PYRO_OK` | **无** |
| `write(p, size)` | DMA 启动失败才 `PYRO_BUSY` | FIFO 不足即 `PYRO_BUSY` | **无**（语义一致） |
| rx 回调粒度 | IDLE 语义（近似一帧） | 驱动内组帧（严格一帧） | **无** |
| 回调上下文 | ISR | USB 任务（`woken = pdFALSE`） | **无**（调用层不感知） |
| 波特率/带宽 | 921600 ≈ 92KB/s | FS bulk ≈ 1MB/s | 变好 |
| 时间分辨率 | 字节级 | 主机 1ms 调度 | `get_com_interval()` 统计分布更"抖动" |
| 上电就绪 | `MX_USARTx_Init()` 后即可用 | 枚举完成后（数百 ms） | 前数百 ms `write()` 返回 `PYRO_ERROR`（被 500ms 延时 + 50ms 超时吸收） |
| PC 侧身份 | 固定 COM（USB-TTL 芯片） | CDC 免驱，COM 号可变 | 上位机需按端口属性匹配 |

结论：**协议层与 API 层对调用层完全等价；物理层特性不同（无波特率、1ms 时间量化、突发更集中）。**

## 附录 B：落地顺序（每步可独立验证/回滚）

| 步 | 内容 | 验证 |
|---|---|---|
| 1 | 建 `tinyusb` target + 4 个新增源文件（回环模式） | 编译通过 |
| 2 | `usb_task_t::init()` 时钟/PHY/GPIO/NVIC + 迁移 `OTG_HS_IRQHandler` + 停用 `HAL_PCD_Init` | 枚举出 COM 口 |
| 3 | 回环 + `mounted`/`dtr` 上报 | 验收表 3-5 项 |
| 4 | 引脚/稳定性确认 | 验收表 6-7 项 |
| 5 | 零回归确认（UART 链路全部照旧） | 验收表 8 项 |
| 6 | （阶段 3）接口接入 + 组帧 + 切宏 | 上位机联调 |

---

## 9. 通用化改造方案（A + B + C）— 已定稿，待实施

> **背景**：§1–§8 已把 USB CDC 接通自瞄链路，但驱动链路中残留机型字样与自瞄自测脚手架，且板级硬件细节内嵌在驱动内。
>
> **目标**：使 USB CDC 成为 **PYRo 框架级通用虚拟串口驱动** —— 不出现任何机型/业务字样，并与 `bsp_uart` / `bsp_can` 的分层约定对齐。
>
> **范围**：A（描述符身份参数化）+ B（清除自瞄残留与自测脚手架）。~~C（抽出 `bsp_usb`）~~ → **已取消**，理由见 §9.5。
>
> **不在范围**：C（抽出 `bsp_usb`）、D（多实例/多路 CDC）、E（跨 MCU 参数化），理由见 §9.5。
>
> **状态**：方案定稿；**落笔时尚未修改任何代码**。

### 9.1 机型指纹盘点（含证据）

| 类别 | 位置 | 内容 | 判定 |
|---|---|---|---|
| 真指纹 | `PYRo/Peripheral/USB/pyro_usb_descriptors.c:76` | 产品名 `"Infantry2 Virtual COM"` | 必须参数化 |
| 真指纹 | 同上 `:77` | 序列号 `"PYRO-INF2-0001"` 固定 → 同 PC 多台机器人被识别为同一设备（§8.13.4） | 改为 UID 派生 |
| 真指纹 | `PYRo/Peripheral/USB/pyro_usb_cdc_drv.cpp:123-126` | `enable_rx()` 自测档位里硬编码自瞄帧 `_parser.configure(0xA5, 29)` | 删除 |
| 真指纹 | `pyro_usb_cdc_drv.cpp:8-10,209-228,255-263,285-313` | `USB_CDC_LOOPBACK` 三档回环 + mount/DTR 回环打印 | 删除（§4 已承诺） |
| 真指纹 | 顶层 `CMakeLists.txt:97-100` | `USB_CDC_SELF_TEST`、死宏 `USB_CDC_STANDALONE_TEST`（代码零引用） | 删除 |
| 注释级 | `pyro_frame_parser.h:22,65` | "自瞄帧 29B"、`_sof{0xA5}` 默认值 | 去机型化 |
| 注释级 | `pyro_serial_itf.h:20-24,33` | 点名 `infantry2_autoaim_drv_t` 为唯一消费者 | 去机型化 |
| 结构（非指纹） | `pyro_usb_cdc_drv.cpp:25-39` | GPIOA PA11/PA12 + `GPIO_AF10_OTG1_HS` + `OTG_HS_IRQn` + NVIC 优先级内嵌驱动 | **保持现状**（目标板固定达妙 MC02、单 USB 口，不做 BSP；见 §9.4/§9.5） |
| 结构（非指纹） | `pyro_usb_cdc_drv.h:28,59` | `instance()` 单例（UART 侧单例归 `bsp_uart`） | **保持现状**（单 USB 口，无多实例需求） |
| 非驱动，正常分工 | `Robot/Infantry2/CMakeLists.txt:65`、`pyro_init_thread.cpp:78-86`、`pyro_autoaim_drv.cpp:39-43,105` | `AUTOAIM_USB_CDC` 宏、`start()+enable_rx()`、`set_frame_config(0xA5,29)` | 保持（消费者接线） |

### 9.2 A｜描述符身份参数化（A-1 方案）

动机：描述符是唯一硬编码机型名的位置；且固定序列号会导致多机冲突。

**A-1 新增 `PYRo/Peripheral/USB/pyro_usb_desc_config.h`**（**定稿已取代本节内容**：不再引入 `PYRO_USB_ROBOT_NAME`/`PYRO_USB_BOARD_ROLE` 与字符串化宏，产品名固定为 `"PYRo Robot Virtual COM"`，序列号由应用层直接书写 —— 见 §9.2.4）：

```cpp
#define PYRO_USB_STR_(x) #x
#define PYRO_USB_STR(x)  PYRO_USB_STR_(x)

#ifndef PYRO_USB_ROBOT_NAME          /* 由 CMake 注入：-DPYRO_USB_ROBOT_NAME=INFANTRY2 */
#define PYRO_USB_ROBOT_NAME PYRo     /* 非 CMake 裸编译时的兜底 */
#endif

#define PYRO_USB_MANUFACTURER_STR "PYRo"
#define PYRO_USB_PRODUCT_STR      PYRO_USB_STR(PYRO_USB_ROBOT_NAME) " Virtual COM"
#define PYRO_USB_CDC_ITF_STR      "PYRo CDC"

#ifndef PYRO_USB_VID
#define PYRO_USB_VID 0xCAFE
#endif
#ifndef PYRO_USB_PID
#define PYRO_USB_PID 0x4010
#endif
#ifndef PYRO_USB_BCD
#define PYRO_USB_BCD 0x0100
#endif
```

**A-2 注入机型名（未采用）**（`ROBOT_NAME` 定义于 `CMake/config/pyro_robot_id_config.cmake:49-78`，由 `ROBOT_ID` 映射。本方案最终**不做 CMake 注入**——决策为"应用层直接书写序列号"，见 §9.2.4/§9.2.2；以下内容仅作为"若将来需由构建系统注入机型名"的参考）：

- **推荐位置**：`CMake/config/pyro_robot_id_config.cmake` 第 107 行之后（紧邻 ID 常量，机器人身份单一来源；`add_compile_definitions` 为目录级，自动覆盖其后 `add_subdirectory(PYRo)` / `add_subdirectory(robot/...)` 的全部目标）。
- **备选位置**：顶层 `CMakeLists.txt:6` 的 `include(...pyro_robot_id_config.cmake)` 之后（同目录作用域，变量可见；但身份定义被拆到两处）。

```cmake
# USB CDC 描述符机型名（无空格标识符；ROBOT_NAME 由本文件/上游按 ROBOT_ID 计算）
if(DEFINED ROBOT_NAME)
    add_compile_definitions(PYRO_USB_ROBOT_NAME=${ROBOT_NAME})
else()
    add_compile_definitions(PYRO_USB_ROBOT_NAME=PYRo)
endif()
```

> 说明：`ROBOT_NAME` 是普通（非 CACHE）变量，未定义 ROBOT_ID 时其值为 `"UNKNOWN"`，此处会得到 `-DPYRO_USB_ROBOT_NAME=UNKNOWN`（可接受，且与构建横幅的显示一致）。

**A-3 `pyro_usb_descriptors.c` 改造**

> **定稿说明（重要，2026-09 实施）**：采用 §9.2.4 定稿形态后，本小节的第 **1、4、5** 步（`stm32h7xx_hal.h`/UID 读取、`fill_serial_utf16()`、回调 switch 化）**均不再需要**。实际实现为：① 厂商/产品/VID/PID 改为编译期常量（`pyro_usb_desc_config.h`）；② `string_desc_arr[]` 第 3 项（Serial）留占位值并**可写**，新增 `void pyro_usb_desc_set_serial(const char *s)` 供驱动调用；③ 序列号由应用层在 `start(serial)` 中**直接书写**（如 `"INFANTRY2-GIMBAL"`），长度/字符集**在 `start()` 里运行期校验**（不引入 `PYRO_USB_SERIAL_STR` 宏，也无 `static_assert`）。下列内容保留为"若将来需要含 UID 序列号"的实现参考。

1. include：增加 `pyro_usb_desc_config.h` 与 `stm32h7xx_hal.h`（取 UID：`HAL_GetUIDw0/1/2()`，见 `stm32h7xx_hal.h:1090-1092`；与 `pyro_usb_cdc_drv.cpp` 引 HAL 的做法一致）；
2. `USB_VID/PID/BCD` 改为引用宏；
3. 字符串表第 3 项（Serial）留空，改由回调运行时生成；
4. 新增序列号生成 `fill_serial_utf16()`：`"<ROBOT_NAME>-" + UID 低 64bit`（8 字节 → 16 hex；最长 `SUB_ENGINEER-` 13+16=29 ≤ 31 上限）；**直接把 ASCII 写成 UTF-16LE 存进描述符缓冲**（USB 字符串描述符即为 UTF-16LE，故无需中间 char 缓冲/转换）；**不引入 printf**（全仓无 `sprintf/snprintf` 使用，手写 hex 与 TinyUSB 上游 `board_usb_get_serial` 同风格）：

```c
static uint8_t fill_serial_utf16(uint16_t *dst, uint8_t cap)
{
    static const char hex[]    = "0123456789ABCDEF";
    static const char prefix[] = PYRO_USB_STR(PYRO_USB_ROBOT_NAME) "-";
    const uint32_t uid[2] = { HAL_GetUIDw0(), HAL_GetUIDw1() };   /* UID 低 64bit */

    uint8_t n = 0;
    for (const char *p = prefix; *p && n < cap; ++p) dst[n++] = (uint16_t)(uint8_t)*p;
    for (int w = 0; w < 2; ++w) {                     /* UID 低 64bit */
        const uint32_t v = uid[w];
        for (int s = 28; s >= 0 && n < cap; s -= 4) dst[n++] = (uint16_t)hex[(v >> s) & 0x0F];
    }
    return n;
}
```

5. `tud_descriptor_string_cb()` 改为 switch 形式（与 TinyUSB 官方示例同构），仅特化 `STRID_SERIAL`：

```c
switch (index) {
case STRID_LANGID:  memcpy(&_desc_str[1], string_desc_arr[STRID_LANGID], 2); chr_count = 1; break;
case STRID_SERIAL:  chr_count = fill_serial_utf16(&_desc_str[1],
                              (uint8_t)(sizeof(_desc_str)/sizeof(_desc_str[0]) - 2)); break;
default:            /* 越界返回 NULL；其余按 string_desc_arr[index] 转 UTF-16LE，cap 31 */ break;
}
_desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
```

> **取数来源**：`HAL_GetUIDw0()/HAL_GetUIDw1()`（`stm32h7xx_hal.h:1090-1092`，内部读 `UID_BASE`；H723 的 `UID_BASE = 0x1FF1E800`，见 `stm32h723xx.h:2082`，96 位 UID 出厂烧录、芯片级唯一，任意字组合均可作序列号）。等价替代写法：直接读 `(const volatile uint32_t *)UID_BASE` 指针（TinyUSB 上游 `hw/bsp/stm32*/family.c:board_get_unique_id()` 即此风格）。
>
> **身份稳定性权衡**：序列号含 `ROBOT_NAME` → 同一块板改刷为别的机型会得到**新的 USB 身份**（新设备实例、新 COM 号）；若希望"身份只随硬件走"，可改为纯 UID（如 `PYRO-1A2B3C4D5E6F7A8B`），机型只体现在产品名。当前方案取"机型名 + UID"，便于在一台 PC 上同时区分机型与板子。
>
> **缓存副作用**：Windows 按 `VID/PID/Serial` 建立设备实例并缓存字符串；改序列号后旧实例可能残留（设备管理器"显示隐藏的设备"可清理），COM 号也可能重新分配。

> 描述符**结构零变化**：`ITF_NUM_*`、端点 `0x81/0x02/0x82`、`TUD_CDC_DESCRIPTOR(..., 4, ...)` 的字符串索引 4 均不变；`_desc_str[32+1]` 保持不变。

> **为什么需要唯一序列号（"没关系 vs 有关系"判据）**：
>
> - **确实无关的场景**：同一时刻只接一块板（拔一块插一块）、上位机写死 COM 号、且始终插同一个 USB 口 —— 此时固定序列号/机型名序列号反而让 COM 号保持恒定，工程上完全可用（现实中大量 CH340/CP2102 等 USB-TTL 芯片就没有唯一序列号）。
> - **会出问题的场景**：① **同一 PC 并发插两块"设备实例相同"的板**（备板、同型多车、同 ROBOT_ID 下的多块板）→ Windows 视为同一设备实例 → COM 号互相覆盖、只能打开其中一个；第二块插入会让第一块"重新枚举"，正在跑的链路掉线（易被误判为"USB 不稳定/抗干扰差"）。② **换 USB 口** → 无唯一序列号时 Windows 按**端口位置**建实例 → COM 号漂移（这正是 USB-TTL 模块"换个口 COM 号就变"的根因）；有唯一序列号则"插哪个口都是同一实例/同一 COM 号"。③ 多机型都接 USB 时，冲突面从"跨机型"扩大到"同机型多板"。
> - **收益/代价**：唯一序列号的收益是"①可并发 ②换口不换号"；代价是"改刷 `ROBOT_ID` 会换身份"（仅"机型+UID"方案有此项，纯 UID 无此代价）。
> - **若上位机必须写死 COM 号**：可用唯一序列号 + 在设备管理器手动固定每块板的 COM 号（端口属性 → 高级），比"依赖插口位置"更稳定；否则建议上位机按 VID/PID + 产品名（机型）或序列号前缀筛选端口。

### 9.2.1 备选方案：车型 + 板别（不使用芯片 UID）

> **定稿说明**：本节提出的"车型+板别"形式**最终被采纳**（见 §9.2.4 定稿决定）；本节其余内容（板别 token 现状、注入方式、能力边界）作为决策依据保留。

动机：若"同机型 + 同板别的多块板并发"不会发生，则序列号可用**纯编译期常量**表达，实现比 A-3 更简单（无需 UID、无需运行时生成与 `cap` 逻辑）。

| 项 | 内容 |
|---|---|
| 身份形态 | `iProduct = "INFANTRY2 GIMBAL Virtual COM"`；`iSerialNumber = "INFANTRY2-GIMBAL"` |
| 数据来源 | **定稿改为应用层直接书写字面量**（不做 CMake 注入，见 §9.2.4）；下表板别 token 仅作参考 |
| 代码改动 | `pyro_usb_descriptors.c` 第 3 项保留占位值 + 新增 `pyro_usb_desc_set_serial()`；启动时由 `start(serial)` 注入 → **A-3 的 `fill_serial_utf16()` 与 UID 读取全部不需要**（`tud_descriptor_string_cb` 保持原索引式写法即可） |
| 长度 | 最长 `SUB_ENGINEER-CHASSIS` = 20 ≤ 31 上限，安全 |

```c
/* pyro_usb_desc_config.h 追加 */
#ifndef PYRO_USB_BOARD_ROLE           /* 由各机型 CMake 注入：GIMBAL / ARM / CHASSIS（无空格标识符） */
#define PYRO_USB_BOARD_ROLE MAIN
#endif
#define PYRO_USB_SERIAL_STR  PYRO_USB_STR(PYRO_USB_ROBOT_NAME) "-" PYRO_USB_STR(PYRO_USB_BOARD_ROLE)
```

板别 token 的现状（证据）：

| 机型 | `BOARD` 取值 | 是否已有板别宏 | 位置 |
|---|---|---|---|
| Infantry2 | `GIMBAL_BOARD` / `CHASSIS_BOARD` | `GIMBAL_BOARD=1`、`CHASSIS_BOARD=2` | `Robot/Infantry2/CMakeLists.txt:4,13-17` |
| Sentry | `GIMBAL_BOARD` / `CHASSIS_BOARD` | 同上 | `Robot/Sentry/CMakeLists.txt:4,13-17` |
| Engineer | `ARM_BOARD` / `CHASSIS_BOARD` | 仅 `BOARD=${BOARD}`（**无** `GIMBAL_BOARD` 宏） | `Robot/Engineer/CMakeLists.txt:4,14-16` |

→ 命名不统一（`GIMBAL` / `ARM` / `CHASSIS`），故两种做法：
1. **推荐**：各机型 CMake 的 `target_compile_definitions` 显式加 `PYRO_USB_BOARD_ROLE=GIMBAL|ARM|CHASSIS`（每机型 1 行，语义统一）；
2. 零改动退路：`#define PYRO_USB_BOARD_ROLE BOARD`，代价是显示为 `INFANTRY2-GIMBAL_BOARD`（略啰嗦但可用；注意 BOARD 未定义时会串化成 `"BOARD"`）。

**能力边界（务必接受）**：该方案把序列号降级为"类型标识"，与 `iProduct` 语义重叠；**无法区分同机型 + 同板别的两块板**（备板、同型第二台车、修板对照）——这两块板仍会抢同一个 COM 号 / 设备实例。同理，`ROBOT_ID=4` 下的云台板与底盘板虽可用板别区分，但"另一台车的云台板"无法区分。

**两全改良（推荐折中）**：前缀保留可读性、尾部保留唯一性 ——
`iProduct = "INFANTRY2 GIMBAL Virtual COM"`，`iSerialNumber = "INFANTRY2-GIMBAL-" + UID 低 32bit（8 hex）`
唯一性由尾部保证，运维仍可从前缀看出车型与板别；代价仅为一次 `HAL_GetUIDw0()`。

**全人工编号（另一种无 UID 的唯一化）**：`INFANTRY2-GIMBAL-01`，由 `-DPYRO_USB_BOARD_SN=01` 或 Flash 配置注入。优点：可读、可与队内资产台账对应；代价：需人工维护，漏改/重复即冲突。

### 9.2.2 运行时注入方案：驱动不读取任何身份来源

> **定稿说明**：运行时注入的**机制**被采纳（应用层通过 `start(serial)` 提供身份，见 §9.2.4），但**范围收敛**为"只注入序列号"；UID 读取与可覆盖产品名**均未采用**——驱动仍不读 UID（写入方是应用层），故本节"驱动零身份依赖"的收益依然成立。

动机：把"身份从哪来"的决策权移到应用层（机型/BSP），驱动只负责"接收并使用"。这样驱动既不依赖 HAL/UID，也不依赖机型宏——与 §9.4 的 BSP 分层最一致，也是"通用化"最彻底的形态。

**分层**

```text
机型层 Robot/<robot>/pyro_init_thread.cpp     ← 决定身份内容（硬编码 / 计算 / 读 Flash / 上位机下发均可）
      │ usb_device_info_t
      ├── PYRo/Peripheral/USB/pyro_bsp_usb.cpp ← 板级：可选提供 UID→序列号 等生成工具
      ▼
usb_cdc_drv_t::set_device_info()               ← 驱动只存指针 + 校验长度（不读 UID、不含机型）
      ▼
pyro_usb_descriptors.c                         ← 描述符回调按需转 UTF-16LE 返回
```

**接口（驱动侧，`pyro_usb_cdc_drv.h`）**

```cpp
struct usb_device_info_t
{
    const char *manufacturer;   // ASCII，静态存储；nullptr → 驱动默认 "PYRo"
    const char *product;        // ASCII，静态存储；nullptr → 驱动默认 "PYRo Virtual COM"
    const char *serial;         // ASCII，静态存储；nullptr → 不带序列号（iSerialNumber = 0）
};

// 必须在 start() 之前调用；字符串须为静态存储且生命周期覆盖整个运行期
status_t set_device_info(const usb_device_info_t &info);
```

- **只注入字符串**；`VID/PID/BCD` 保持编译期宏 → `desc_device` 仍为 `const`（留在 Flash，零 RAM 增量）。
- `pyro_usb_descriptors.c` 中 `string_desc_arr[]`（指针数组本身**可写**）→ 注入即改写第 1/2/3 项指针；返回前统一做 ASCII→UTF-16LE 与长度（≤31 字符）校验。

**时序与生命周期约束（必须遵守）**

1. **注入先于 `tusb_init()`**：调用链为 `start()` → `usb_task_t::init()` → `run_loop()` 内 `tusb_init()`；枚举只发生在 `tusb_init()` 之后，故在 `start()` 之前注入即无竞态。
2. `tud_descriptor_*_cb` 在 USB 任务上下文（`tud_task()` 内）被调用，且会被**多次**调用（重复 GET_DESCRIPTOR、重枚举）→ 传入字符串**不得是栈对象或临时 `std::string`**，必须是静态存储。
3. 运行期再次注入无效（Windows 已缓存枚举结果）→ 约定"**仅在 `start()` 前调用一次**"。

**推荐调用范式（C 取消后：机型层直接调用驱动，两步与 UART 分支同构）**：

```cpp
// 机型层（pyro_init_thread.cpp）
usb_cdc_drv_t::instance().start("INFANTRY2-GIMBAL");   // 序列号由应用层自行书写（§9.2.4）

// 与 UART 分支一一对应：reset(baud,...) ↔ start(serial)；enable_rx_dma() ↔ enable_rx()
usb_cdc_drv_t::instance().enable_rx();
```

**UID 读取的归属（已不采用）**：定稿序列号不含 UID；若将来需要，`HAL_GetUIDw0()` 也应放在机型层/应用层工具函数中，**不在 CDC 驱动内**。

**收益 / 代价**

| 收益 | 代价 |
|---|---|
| 驱动零身份依赖（无机型宏、不读 UID、不引 HAL）→ 跨机型/跨平台复用最干净 | 多一个初始化契约（必须在 `start()` 前注入） |
| 身份可运行时决定：读 Flash 配置 / 人工编号 / 上位机下发 / 多产品形态 | 忘记注入会静默落到默认身份 → 多板并发隐患，需显式失败或编译期提示 |
| 与将来 D（多实例/多路 CDC）天然契合（C 已取消，不再涉及 BSP） | 描述符文件从"纯常量"变为"可写指针数组"（极小改动） |

**"忘记注入"的处理（建议）**：`start()` 在未注入时返回 `PYRO_PARAM_ERROR`，强制显式声明身份（避免静默默认值引发"多板抢 COM 号"这类难查故障）；自测/回环场景可在测试宏下放行默认值。

**与 §9.2 编译期宏方案的关系（定稿：不采用宏注入）**：**不使用** `PYRO_USB_ROBOT_NAME` / `PYRO_USB_BOARD_ROLE`，**不做任何 CMake 注入**，也不提供 `PYRO_USB_SERIAL_STR`；序列号由应用层在 `start(serial)` 处**直接书写**（如 `start("INFANTRY2-GIMBAL")`），唯一性由调用者负责。驱动侧不需要 `fill_serial_utf16()`。C（BSP）已取消，调用点即机型层 `pyro_init_thread.cpp`。

### 9.2.3 验证与风险（承 A-1~A-3；与所选序列号形式无关）

**A-4 验证**：以 `ROBOT_ID=4` / `ROBOT_ID=5`（或切 `BOARD`）各编译一次 → 设备管理器中**产品名恒为** `PYRo Robot Virtual COM`，序列号为 `<ROBOT_NAME>-<BOARD_ROLE>`；不同机型/板别的两块板同插 → 两个独立实例。若 A-3 改走含 UID 的参考实现，则序列号额外带 UID 尾部。

**A-5 风险**：产品名/序列号变化会使 Windows 重新分配 COM 号，上位机需按 VID/PID 或产品名匹配（见 §6.4）；跨 MCU 时 `UID_BASE` 需 `#if defined(UID_BASE)` 兜底。后续若需更长序列号（96bit UID），受 31 字符上限约束，需同时评估描述符缓冲大小。

### 9.2.4 精简定稿形态（推荐）：厂商/产品编译期固定，仅序列号由应用层注入

| 描述符字段 | 取值 | 决定方式 |
|---|---|---|
| `iManufacturer` (index 1) | `"PYRo"` | 编译期常量 |
| `iProduct` (index 2) | `"PYRo Robot Virtual COM"` | 编译期常量（**不含机型** → 对所有机型/板别一致） |
| `iSerialNumber` (index 3) | `INFANTRY2-GIMBAL`（**车型+板别，不含 UID**） | **仅此一项**由应用层注入（编译期常量亦可） |
| `VID / PID / BCD` | `0xCAFE / 0x4010 / 0x0100` | 编译期宏 |

**接口收敛为"只设序列号"**：

```cpp
class usb_cdc_drv_t final : public serial_itf_t
{
  public:
    /// 启动 USB 设备栈。serial 必填：nullptr/空串 → 返回 PYRO_PARAM_ERROR。
    /// @param serial 可打印 ASCII、非空、≤31 字符；由应用层自行书写（如 "INFANTRY2-GIMBAL"）
    status_t start(const char *serial);
};
```

**为什么这样收敛是合理的**

1. Windows 的设备去重**只看 `VID/PID/Serial`**，`iProduct` 仅影响显示名称 → 把可变部分收敛到 serial，区分能力不损失。
2. 厂商/产品成为编译期常量 → 描述符文件几乎不必从"纯常量"改为"可写数组"，只有 index 3 一项可变；`desc_device` 保持 `const`（留在 Flash，零 RAM 增量）。
3. 对外契约只剩一个参数 → "忘记注入/半注入"的隐患面大幅缩小。

**必须配套的三条约束**

| # | 约束 | 理由 |
|---|---|---|
| 1 | **serial 必填**（不提供"无序列号"分支） | `desc_device.iSerialNumber` 固定为 `0x03`；允许空串会让主机收到 0 长度字符串描述符（行为不可预期）；且产品名固定后，"无序列号"会退化为按端口位置建实例 → 换口变 COM 号、多板必撞 |
| 2 | **≤31 字符且为可打印 ASCII**；首选**编译期 `static_assert`**（序列号为宏常量时），运行期再兜底一次（超限返回 `PYRO_PARAM_ERROR`，禁止静默截断） | 本项目缓冲为 `_desc_str[32+1]`；截断会造成"两块板序列号相同"这种最危险的静默失败 |
| 3 | **静态存储 + 在 `start()` 之前提供** | `tud_descriptor_*_cb` 会被多次调用（重枚举、重复 GET_DESCRIPTOR） |

**可读性如何保留**：产品名固定后，机型/板别信息由**序列号**承载 → 设备实例路径仍可读：`USB\VID_CAFE&PID_4010\INFANTRY2-GIMBAL`。

**已知取舍**：设备管理器的"设备名"对所有机型/板别都显示同一串 `PYRo Robot Virtual COM`，多机型同插时只能靠 COM 号或序列号区分（这是产品名统一的必然结果）。

**演进留口**：若将来确需把机型写进产品名，只需把 `product` 纳入 `usb_device_info_t` 作为**可选覆盖**（nullptr → 用固定默认值），机制与调用点不变。

**定稿决定（本次确认）：序列号 = `车型+板别`，不含 UID**

| 维度 | 结论 |
|---|---|
| 形态 | 应用层**直接书写的字符串字面量**，例 `"INFANTRY2-GIMBAL"`（不引入任何 CMake 宏注入） |
| 来源 | 应用层（机型 `pyro_init_thread.cpp`）；驱动只负责校验与注入，不推导任何机型信息 |
| 长度 | 建议 ≤ 31 字符；由 `start()` **运行期校验**（超长返回 `PYRO_PARAM_ERROR`，不静默截断） |
| 不再需要 | UID 读取、`make_serial()` 十六进制格式化、`stm32h7xx_hal.h` 依赖、运行期长度兜底（可保留一次断言） |
| 附带收益 | 同一机型+板别的板子（含备板）插任意 USB 口都是**同一设备实例 / 同一 COM 号** → 上位机写死 COM 号依然可用 |
| 唯一限制（须接受） | 同一 PC **并发**插两块"同机型+同板别"的板 → 视为同一设备：COM 号互相覆盖、只能打开其中一个、第二块插入可能让第一块重新枚举（判据见本节前的"没关系 vs 有关系"清单） |

### 9.3 B｜清除自瞄残留与自测脚手架

| 位置 | 动作 |
|---|---|
| `pyro_usb_cdc_drv.cpp:8-10` | 删 `USB_CDC_LOOPBACK` 默认值块 |
| `pyro_usb_cdc_drv.cpp:121-129` | `enable_rx()` 删 `_parser.configure(0xA5, 29)` 自测分支，仅留 `_rx_enabled = true; return PYRO_OK;` |
| `pyro_usb_cdc_drv.cpp:209-228` | `dispatch()` 删自测档位 2（无接收方回环）整块 |
| `pyro_usb_cdc_drv.cpp:255-263` | 删 `loopback_write()` 定义 |
| `pyro_usb_cdc_drv.cpp:265-283` | `on_cdc_rx()` 固定走 `dispatch()` |
| `pyro_usb_cdc_drv.cpp:285-292` | `on_mount()` 置空 + 注释"预留挂载钩子（当前无业务动作）" |
| `pyro_usb_cdc_drv.cpp:300-313` | `on_line_state()` 仅保留 `(void)dtr; (void)rts;`（预留 DTR 策略钩子） |
| `pyro_usb_cdc_drv.h:90-91` | 删 `loopback_write()` 声明 |
| 顶层 `CMakeLists.txt:95-100` | 删 `USB_CDC_SELF_TEST` 段（含代码零引用的 `USB_CDC_STANDALONE_TEST`） |
| `pyro_frame_parser.h:22,65` | 注释去机型化（"自瞄帧 29B" → "常见业务帧约 29B"）；`_sof{0xA5}` → `_sof{0x00}`（`enabled()` 由 `_len` 决定，默认 SOF 不影响行为） |
| `pyro_serial_itf.h:20-24,33` | 注释去具名消费者（`infantry2_autoaim_drv_t` → "抽象消费者"） |
| **保留** | `is_mounted()` / `is_connected()` / `on_umount()`（`_parser.reset()` + `tud_cdc_read_flush()` 为必需逻辑） |

**B 验证**：`Select-String -Path .\PYRo,.\Robot,.\CMakeLists.txt -Pattern 'USB_CDC_LOOPBACK|USB_CDC_SELF_TEST|USB_CDC_STANDALONE_TEST' -Recurse` → 0 命中（`plan.md` 属历史设计文档，不计）；编译通过；`.map` 内 `cdc_device` / `dcd_dwc2` / `pyro_usb_cdc_drv` 仍在。

### 9.4 C｜抽出 `bsp_usb`，对齐框架 BSP 约定（**已取消，保留为决策记录**）

> **定稿说明（C 取消）**：
>
> 1. **目标板固定为达妙 MC02，只有一路 USB**，不存在"多实例统一管理"需求——而 UART 引入 BSP 的首要动因正是要管理 UART1/5/7/10 多路实例（`pyro_bsp_uart.cpp:20-44` 的 `get_uartN()`）。
> 2. 板级细节（PA11/PA12 + `GPIO_AF10_OTG1_HS` + `OTG_HS_IRQn`）在本工程是**唯一且确定**的事实，留在驱动 `usb_task_t::init()` 内即可；换板时只改一处。
> 3. 取消后调用侧与现状一致：`usb_cdc_drv_t::instance().start(serial)` + `enable_rx()`，改动面更小、无需新增 `pyro_bsp_usb.*`。
>
> **演进留口**：若将来出现第二路 USB 口，或换到 OTG_FS / ULPI 外部 PHY / 其他 MCU，再按本节做法抽出 BSP（或改用 ops 注入）。下方内容即为那时的可用方案。

对齐依据：UART 为 `pyro_bsp_uart`（持单例；`uart_drv_t` 构造私有 + `friend class bsp_uart`）、CAN 为 `pyro_bsp_can`；USB 目前板级细节内嵌驱动、无对应 BSP。

**C-1 新增 `PYRo/Peripheral/USB/pyro_bsp_usb.h`**

```cpp
class bsp_usb
{
public:
    static status_t hw_init();            // DP/DM 引脚(AF) + 中断优先级；须在 tusb_init() 之前
    static usb_cdc_drv_t &get_cdc();      // 与 bsp_uart::get_uart1() 同构
};
```

**C-2 新增 `PYRo/Peripheral/USB/pyro_bsp_usb.cpp`**：`get_cdc()` 返回局部静态实例；`hw_init()` 为 `usb_task_t::init()` 原文搬迁（GPIOA PA11/PA12 + `GPIO_AF10_OTG1_HS` + `OTG_HS_IRQn` NVIC 优先级），保留原有"USB 时钟源/电压检测由 CubeMX `HAL_PCD_MspInit()` 维护、切勿硬编码 `RCC_USBCLKSOURCE_*`"注释。

**C-3 `pyro_usb_cdc_drv.h`**：删 `instance()`；构造/析构保持私有并新增 `friend class bsp_usb;`；类注释注明"实例由 `bsp_usb::get_cdc()` 持有；本类不含板级 GPIO/IRQ 细节"。

**C-4 `pyro_usb_cdc_drv.cpp`**：include `pyro_bsp_usb.h`；`usb_task_t::init()` 改为 `return bsp_usb::hw_init();`；删除 `instance()` 定义（`:77-81`）；C 桥接 4 处（`:320-349`）改调 `bsp_usb::get_cdc()`；`run_loop()`（`tusb_init` + `tud_task`）不变。

**C-5 调用侧（2 文件 4 处）**：`Robot/Infantry2/pyro_init_thread.cpp:9,82,85`、`Robot/Infantry2/Communication/Gimbal_board/pyro_autoaim_drv.cpp:6,41` —— 改为 `#include "pyro_bsp_usb.h"` + `bsp_usb::get_cdc()`。

**C-6 `PYRo/CMakeLists.txt`**：源列表新增 `Peripheral/USB/pyro_bsp_usb.cpp`（include 路径已含 `Peripheral/USB`，无需再加）。

**C 取舍（推荐 C-1 式）**：drv 直接调 `bsp_usb::hw_init()`，代价是 `pyro_usb_cdc_drv.cpp` 出现 drv → bsp 的编译期依赖（UART 侧无此依赖，但 `uart_drv_t` 签名里本就带 `UART_HandleTypeDef*`，属同类"松散分层"）。若要求严格单向依赖（bsp → drv），可升级为 **ops 注入**：`struct usb_hw_ops_t { status_t (*hw_init)(); };` + `explicit usb_cdc_drv_t(const usb_hw_ops_t&)`，由 bsp 构造时传入。建议先按 C-1 落地（行为零变化、可回滚），真上第二个平台时再升级。

**C 验证（决策记录，本次不实施）**：若将来实施——编译通过且 `pyro_bsp_usb.cpp` 出现在 `compile_commands.json`；枚举与自瞄收发回归正常；`Select-String -Path .\PYRo,.\Robot -Pattern 'OTG_HS_IRQn|GPIO_AF10_OTG1_HS' -Recurse` → 仅命中 `pyro_bsp_usb.cpp`。

### 9.5 本次不做（C/D/E）与理由

| 项 | 内容 | 不做的理由 |
|---|---|---|
| C | 抽出 `bsp_usb`（板级 GPIO/NVIC 下沉 + 单例归属 BSP） | 目标板固定达妙 MC02、**单 USB 口**，无多实例管理需求（UART 引入 BSP 的首要动因即"多串口统一管理"）；板级细节唯一且确定，留在驱动内即可；取消后调用侧与现状一致、改动更小（§9.4 保留为决策记录与演进留口） |
| D | 多实例 / 多路 CDC（`CFG_TUD_CDC>1`、C 回调按 `itf` 分发、多份 CDC 描述符） | 当前 `MAX_RX_CALLBACKS=4` + `owner` 已支持"一路链路多消费者"；多路 CDC 会新增 COM 口、改变描述符字符串索引与上位机匹配规则、增加 FS 带宽与中断负担。真出现"一台设备同时开两路 CDC"的需求再评估 |
| E | `CFG_TUD_MAX_SPEED` / `BOARD_TUD_RHPORT` / `CFG_TUSB_MCU` 参数化以支持跨 MCU | 仓库当前仅 STM32H723；保留现有 `#error` 策略（宁可编译失败，也不要静默不枚举） |

> 若将来要做 D，最小改动点是：`usb_cdc_drv_t` 去单例 → 内部按 `CFG_TUD_CDC` 组实例数组；C 桥接用 `itf` 参数索引实例（现 `tud_cdc_rx_cb` / `tud_cdc_line_state_cb` 均为 `(void) itf`）；`tusb_config.h` 与 `pyro_usb_descriptors.c` 同步扩到 2 路（端点 `0x83/0x04/0x84`）。

### 9.6 实施顺序、验证与回滚

| 序 | 内容 | 验证 | 回滚 |
|---|---|---|---|
| 1 | B（清残留；4 文件） | 三宏 grep 0 命中 + 编译通过 + 自瞄链路回归 | `git revert` |
| 2 | A（1 新增 + 3 修改：desc-config、descriptors、驱动 `start(serial)`、调用侧序列号字面量） | 编译通过；设备管理器产品名恒为 `PYRo Robot Virtual COM`、序列号 = 应用层所写的字面量（本工程 `INFANTRY2-GIMBAL`） | `git revert`（COM 号会再变一次） |
| ~~3~~ | ~~C（2 新增 + 5 修改 + CMake）~~ **已取消**（见 §9.4/§9.5） | — | — |

> 顺序理由：先 B 消掉自瞄残留，使 A 的 diff 不与回环代码交织；C 已取消，故本次只剩两步。

### 9.7 验收清单

1. `grep -rn "Infantry2\|INF2\|0xA5, 29\|USB_CDC_LOOPBACK" PYRo/` → 0 命中（框架层无机型/业务字样）。
2. 板级细节仍**只存在于** `pyro_usb_cdc_drv.cpp` 的 `usb_task_t::init()`（PA11/PA12 + `GPIO_AF10_OTG1_HS` + `OTG_HS_IRQn`）—— C 取消后不新增 `pyro_bsp_usb.*`，本项与 §9.1 的"保持现状"判定一致。
3. 产品名恒为 `PYRo Robot Virtual COM`；序列号 = 应用层书写值（本工程 `INFANTRY2-GIMBAL`）→ 换机型/板别只需改这一处字面量；序列号不同的两块板同插 → 两个独立实例；**序列号相同并发则按 §9.2.4 的已知限制**（此项预期不通过）。
4. USB 自瞄链路回归：枚举、收发、50ms 超时在线判定、`get_com_interval()` 统计均正常。
5. 代码规模：删回环/自测（`USB_CDC_LOOPBACK` 相关约 40 行）+ 无新增 `pyro_bsp_usb.*`；新增仅 `pyro_usb_desc_config.h`（约 20 行）与少量注入代码 → **净减约 30~40 行**。
6. 不变项确认：`CubeMX/Core/Src/usb_otg.c`（`HAL_PCD_Init` 宏化）、`stm32h7xx_it.c`（`OTG_HS_IRQHandler` 转发）、`tusb_config.h`（FS/RHPORT/CDC 数量）本次均不动。

### 9.8 文档一致性备注

本方案落地后，§3.7（`enable_rx` 自测档位）、§3.9.1（`USB_CDC_SELF_TEST` 段）、§4 末行（清理测试开关）、§8.13.4（固定序列号）将不再与代码一致；届时在本节下方追加"落地结果"小节即可，**不回溯改写 §1–§8**（保留设计历史与决策依据）。

> 待确认项（实施前需拍板）：仅剩 **④ 是否同步在 `plan.md` 记录落地结果**。**①②③ 均已定稿**：C（BSP）取消（§9.4/§9.5）；序列号 = `车型+板别`（不含 UID，§9.2.4）；身份由应用层经 `start(serial)` 注入。
>
> **收敛提案（定稿，已实施）**：采用 **§9.2.4 精简定稿形态** —— 厂商/产品编译期固定（`"PYRo"` / `"PYRo Robot Virtual COM"`），**仅序列号**由应用层注入且**必填**（≤31 字符、可打印 ASCII）；序列号由应用层**直接书写**（例 `start("INFANTRY2-GIMBAL")`），**不做 CMake 注入、不含 UID**；长度/字符集由 `start()` 运行期校验。**残留限制**：同一 PC 并发插两块"同序列号"的板仍会冲突（COM 号覆盖/只能开一个），判据见 §9.2.1 与"没关系 vs 有关系"清单。
