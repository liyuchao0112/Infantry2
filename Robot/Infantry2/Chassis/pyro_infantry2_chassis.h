#ifndef __PYRO_INFANTRY2_CHASSIS_H__
#define __PYRO_INFANTRY2_CHASSIS_H__

#include "pyro_module_base.h"
#include "pyro_motor_base.h"
#include "pyro_algo_pid.h"

namespace pyro {

struct infantry2_chassis_cmd_t final : public cmd_base_t {
    bool is_enable;

    bool follow_yaw;
    bool is_spin;
    float vx, vy, wz;
};

struct infantry2_chassis_deps_t {
    struct motor_deps_t {
        motor_base_t *rudder[4]{nullptr};
        motor_base_t *wheel[4]{nullptr};
        motor_base_t *yaw{nullptr};
    };

    struct pid_deps_t {
        pid_t *rud_pos_pid[4]{nullptr};
        pid_t *rud_spd_pid[4]{nullptr};
        pid_t *wheel_pid[4]{nullptr};
        pid_t *yaw_follow_pid;
    };

    motor_deps_t *motor{};
    pid_deps_t *pid{};
};

struct infantry2_chassis_data_t {

};

struct infantry2_chassis_ctx_t {
    infantry2_chassis_deps_t deps;
    infantry2_chassis_data_t data;
    infantry2_chassis_cmd_t *cmd{};
};

struct infantry2_chassis_module_params_t {
    using CmdType = infantry2_chassis_cmd_t;
    using ModuleDeps = infantry2_chassis_deps_t;
    using ModuleCtx = infantry2_chassis_ctx_t;
};

class infantry2_chassis_t final
        : public module_base_t<infantry2_chassis_t, infantry2_chassis_module_params_t> {
    friend class module_base_t<infantry2_chassis_t, infantry2_chassis_module_params_t>;
};

}
#endif // __PYRO_INFANTRY2_CHASSIS_H__