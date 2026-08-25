#include "pyro_infantry2_chassis.h"

namespace pyro {

void infantry2_chassis_t::fsm_active_t::state_spin_t::enter(owner *owner) {}

void infantry2_chassis_t::fsm_active_t::state_spin_t::execute(owner *owner) {
    _spin_solve(&owner->_ctx);
}

void infantry2_chassis_t::fsm_active_t::state_spin_t::exit(owner *owner) {

}

} // namespace pyro