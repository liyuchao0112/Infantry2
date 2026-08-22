#include "pyro_core_def.h"
#include "pyro_core_config.h"
#include "FreeRTOS.h"
#include "task.h"

extern "C" {
    extern void pyro_init_thread(void *argument);
    extern void start_debug_task(void *arg);

    void start_mission_planer_task(void const *argument) {
        xTaskCreate(pyro_init_thread, "pyro_init_thread", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);

        vTaskDelete(nullptr);
    }
}