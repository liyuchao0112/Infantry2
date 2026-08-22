# =========================================================================
# 声明Infantry2目标文件库（OBJECT库，仅编译目标文件不链接，保留原工程逻辑）
# =========================================================================
add_library(Infantry2 OBJECT)

target_sources(Infantry2
        PRIVATE
        # === Application ===

        # === Gimbal 模块 ===

        # === Booster 模块 ===
        
        pyro_mission_planer.cpp
        pyro_init_thread.cpp
    )

# ===================== 头文件包含路径（覆盖所有模块根目录） =====================
target_include_directories(Infantry2
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
    )

# ===================== 链接依赖库（保留原工程依赖，可按需增删） =====================
target_link_libraries(Infantry2
        PUBLIC
        stm32cubemx
        dsppp_lib
        PYRo
    )

target_compile_definitions(PYRo PUBLIC
        DR16_UART=PYRO_UART5
        # REFEREE_UART=PYRO_UART1
        # AUTOAIM_UART=PYRO_UART10
        VT03_UART=PYRO_UART7
        # JCOM_DEBUG_PORT=PYRO_UART10
        # VOFA_DEBUG_EN=1
        # VOFA_DEBUG_PORT=PYRO_UART10
)