/** @file
Implementation of actuator.h
*/
#include "stokesim/physics/actuator.h"
#include "stokesim/physics/world.h"

namespace stokesim {
namespace physics {

//////////////////////////////////////////////////

Actuator::Actuator(YAML::Node const& config, Craft const* container) :
    craft(container),
    type(yget("type", config).as<Str>()),
    force(0, 0, 0),
    torque(0, 0, 0),
    power(0),
    wind(0, 0, 0),
    killed(false) {
}

//////////////////////////////////////////////////

void Actuator::Cmd::merge(Cmd const& other) {
    values.insert(other.values.begin(), other.values.end());
    if(other.kill.given()) {kill = other.kill;}
}

//////////////////////////////////////////////////

void Actuator::update(Cmd const& cmd) {
    // Update kill status
    if(cmd.kill.given()) {killed = cmd.kill;}
    // If killed use passive command
    if(killed) {
        kill();
        update_impact(Cmd());
    // Otherwise behave normally
    } else {
        update_impact(cmd);
    }
}

//////////////////////////////////////////////////

} // namespace physics
} // namespace stokesim
