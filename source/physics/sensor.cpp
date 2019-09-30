/** @file
Implementation of sensor.h
*/
#include "stokesim/physics/sensor.h"
#include "stokesim/physics/world.h"

namespace stokesim {
namespace physics {

//////////////////////////////////////////////////

Sensor::Sensor(YAML::Node const& config, Craft const* container) :
    craft(container),
    type(yget("type", config).as<Str>()),
    freq(yget("freq", config).as<Escal>()),
    period(freq>EPS ? 1/freq : INF),
    timestamp(-INF),
    sequence_number(0),
    killed(false) {
}

//////////////////////////////////////////////////

void Sensor::Cmd::merge(Cmd const& other) {
    values.insert(other.values.begin(), other.values.end());
    if(other.kill.given()) {kill = other.kill;}
}

//////////////////////////////////////////////////

void Sensor::update(Cmd const& cmd) {
    // Update kill status
    if(cmd.kill.given()) {killed = cmd.kill;}
    if(killed) {return;}
    // If time since last new measurement is more than measurement period, or if interrupt
    Escal elapsed = craft->world->clock.t - timestamp;
    if(period-elapsed <= 1e-6 || cmd.interrupt) {
        // Evolve internal states (like noise / biases) for all the time that passed
        if(elapsed < INF) {update_states(cmd, elapsed);}
        // Compute latest measurements
        update_measurements(cmd);
        // If allowed, set timestamp to current sim time and update the sequence counter
        if(cmd.update_sequence) {
            timestamp = craft->world->clock.t;
            ++sequence_number;
        }
    }
}

//////////////////////////////////////////////////

} // namespace physics
} // namespace stokesim
