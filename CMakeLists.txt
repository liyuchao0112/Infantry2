cmake_minimum_required(VERSION 3.22)


# 设置机器人ID
# 包含机器人配置（定义ROBOT_NAME）
include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/config/pyro_robot_id_config.cmake)

# ==============================================================================
# 编译器与工具链配置
# 必须在第一次调用 project() 之前包含
# ==============================================================================
set(CMAKE_SYSTEM_NAME Generic)
include(${CMAKE_CURRENT_SOURCE_DIR}/CubeMX/cmake/gcc-arm-none-eabi.cmake)

# ==============================================================================
# 2. 项目定义
# ==============================================================================
set(CMAKE_PROJECT_NAME PYRo_Robot)
project(${CMAKE_PROJECT_NAME} C CXX ASM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif ()
# ==============================================================================
# 3. 额外性能优化 (保留工具链未包含的指令)
# ==============================================================================
add_compile_options(
        "-mthumb"
        "-Wdouble-promotion"
        "-funsafe-math-optimizations"
)
# ==============================================================================
# 4. CMSIS-DSP & dsppp 库引入
# ==============================================================================
set(DSP_REPO "${CMAKE_CURRENT_SOURCE_DIR}/third_party/CMSIS-DSP")

add_compile_definitions(
        ARM_MATH_CM7
        ARM_MATH_LOOPUNROLL
)

set(CMSISDSP_BUILD_EXAMPLES OFF CACHE BOOL "Disable examples" FORCE)
add_subdirectory(${DSP_REPO} build_cmsis_dsp)

target_include_directories(CMSISDSP PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/CubeMX/Drivers/CMSIS/Include"
)

add_library(dsppp_lib INTERFACE)
target_include_directories(dsppp_lib INTERFACE
        ${DSP_REPO}/dsppp/Include
        ${DSP_REPO}/Include
)

# 先创建可执行文件目标，再进入子目录（因为子目录中的配置需要操作这个目标）
add_executable(${CMAKE_PROJECT_NAME})

# ==============================================================================
# 4.5 TinyUSB（USB CDC 虚拟串口）
# ==============================================================================
# 注意：CubeMX 目标必须先创建 —— tinyusb 通过 target_link_libraries 从它获取
#       HAL / CMSIS / FreeRTOS 的头文件路径。
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/CubeMX/cmake/stm32cubemx)

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

# ---- USB CDC 自测开关（默认关闭，生产环境不要打开）----
# USB_CDC_LOOPBACK: 1=字节回环  2=经驱动层组帧后回环（见 PYRo/Peripheral/USB/pyro_usb_cdc_drv.cpp）
option(USB_CDC_SELF_TEST "USB CDC 回环自测（仅调试用）" OFF)
if (USB_CDC_SELF_TEST)
    add_compile_definitions(USB_CDC_LOOPBACK=2 USB_CDC_STANDALONE_TEST=1)
endif()

add_subdirectory(PYRo)

if (ROBOT_ID EQUAL HERO_ID)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/robot/hero)
    target_link_libraries(${CMAKE_PROJECT_NAME}
            Hero)
elseif (ROBOT_ID EQUAL SUB_HERO_ID)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/robot/sub_hero)
    target_link_libraries(${CMAKE_PROJECT_NAME}
            Sub_Hero)
elseif (ROBOT_ID EQUAL INFANTRY1_ID)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/robot/infantry1)
    target_link_libraries(${CMAKE_PROJECT_NAME}
            Infantry1)
elseif (ROBOT_ID EQUAL INFANTRY2_ID)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/robot/infantry2)
    target_link_libraries(${CMAKE_PROJECT_NAME}
            Infantry2)
elseif (ROBOT_ID EQUAL SENTRY_ID)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/robot/sentry)
    target_link_libraries(${CMAKE_PROJECT_NAME}
            Sentry)
elseif (ROBOT_ID EQUAL UAV_ID)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/robot/uav)
    target_link_libraries(${CMAKE_PROJECT_NAME}
            UAV)
elseif (ROBOT_ID EQUAL DARTS_ID)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/robot/darts)
    target_link_libraries(${CMAKE_PROJECT_NAME}
            Darts)
elseif (ROBOT_ID EQUAL RADAR_ID)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/robot/radar)
    target_link_libraries(${CMAKE_PROJECT_NAME}
            Radar)
elseif (ROBOT_ID EQUAL SUB_HERO_ID)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/robot/sub_hero)
    target_link_libraries(${CMAKE_PROJECT_NAME}
            Sub_Hero)
elseif (ROBOT_ID EQUAL SUB_ENGINEER_ID)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/robot/sub_engineer)
    target_link_libraries(${CMAKE_PROJECT_NAME}
            Sub_Engineer)
elseif (ROBOT_ID EQUAL SUB_INFANTRY_ID)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/robot/sub_infantry)
    target_link_libraries(${CMAKE_PROJECT_NAME}
            Sub_Infantry)
elseif (ROBOT_ID EQUAL SUB_SENTRY_ID)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/robot/sub_sentry)
    target_link_libraries(${CMAKE_PROJECT_NAME}
            Sub_Sentry)
elseif (ROBOT_ID EQUAL TEST_ROBOT_ID)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/robot/test_robot)
    target_link_libraries(${CMAKE_PROJECT_NAME}
            Test_Robot)
endif ()


target_link_libraries(${CMAKE_PROJECT_NAME}
        PYRo
        stm32cubemx
        CMSISDSP
        dsppp_lib
)


add_dependencies(echo_robot_id ${CMAKE_PROJECT_NAME})