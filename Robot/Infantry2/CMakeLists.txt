add_library(Infantry2 OBJECT)

target_compile_definitions(Infantry2 PRIVATE
    BOARD=${BOARD}
    GIMBAL_BOARD=1
    CHASSIS_BOARD=2
)

if(BOARD STREQUAL "GIMBAL_BOARD")
    target_sources(Infantry2
        PRIVATE
        # === Application ===
        Application/Gimbal_board/pyro_booster_app.cpp
        Application/Gimbal_board/pyro_gimbal_app.cpp

        # === Gimbal 模块 ===
        Gimbal/pyro_infantry2_gimbal.cpp
        Gimbal/fsm/pyro_infantry2_gimbal_passive_state.cpp
        Gimbal/fsm/pyro_infantry2_gimbal_active_state.cpp
        Gimbal/fsm/pyro_infantry2_gimbal_manual_state.cpp
        Gimbal/fsm/pyro_infantry2_gimbal_auto_state.cpp

        # === Chassis 模块 ===

        # === Booster 模块 ===
        Booster/pyro_infantry2_booster.cpp
        Booster/fsm/pyro_infantry2_booster_passive_state.cpp
        Booster/fsm/pyro_infantry2_booster_active_state.cpp
        Booster/fsm/pyro_infantry2_booster_waiting_state.cpp
        Booster/fsm/pyro_infantry2_booster_ready_state.cpp
        Booster/fsm/pyro_infantry2_booster_single_state.cpp
        Booster/fsm/pyro_infantry2_booster_continue_state.cpp
        Booster/fsm/pyro_infantry2_booster_cali_reverse_state.cpp
        Booster/fsm/pyro_infantry2_booster_cali_forward_state.cpp

        # === Communication ===

        pyro_mission_planer.cpp
        pyro_init_thread.cpp
    )

    # 针对 Gimbal 的特殊头文件包含
    target_include_directories(Infantry2
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/Gimbal
        ${CMAKE_CURRENT_SOURCE_DIR}/Booster
    )

    target_compile_definitions(PYRo PUBLIC
        DR16_UART=PYRO_UART5
        VT03_UART=PYRO_UART1
        AUTOAIM_UART=PYRO_UART7
    )

elseif (BOARD STREQUAL "CHASSIS_BOARD")
    target_sources(Infantry2
        PRIVATE
        # === Application ===


        # === Chassis 模块 ===
        Chassis/pyro_infantry2_chassis.cpp

        # === Communication ===

        pyro_mission_planer.cpp
        pyro_init_thread.cpp
    )

    # 针对 Chassis 的特殊头文件包含
    target_include_directories(Infantry2
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/Chassis
    )

    target_compile_definitions(PYRo PUBLIC
        REFEREE_UART=PYRO_UART1
        # SUPERCAP_UART=PYRO_UART7
    )
endif()

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