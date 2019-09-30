/** @file
Implementation of clock.h
*/
#include "stokesim/physics/clock.h"
#include "stokesim/physics/world.h"

namespace stokesim {
namespace physics {

//////////////////////////////////////////////////

void Clock::Cmd::merge(Cmd const& other) {
    // If a positive timestep command was given
    if(other.dt > 0) {
        // Make sure that timestep command was not already made positive by a different commander
        if(dt > 0) {throw std::runtime_error("More than one Cmd is trying to drive Clock::dt.");}
        // Assign timestep command
        dt = other.dt;
    }
}

//////////////////////////////////////////////////

Clock::Clock(YAML::Node const& config, World const* container) :
    world(container),
    dt0(yget("dt0", config).as<Escal>()),
    date(yget("date", config, 2020.0587).as<Escal>()),
    dt(dt0),
    t(0) {
    // Make sure the provided integration timestep is more than numeric zero
    if(dt < EPS) {throw std::runtime_error("Timestep Clock::dt set too small.");}
}

//////////////////////////////////////////////////

void Clock::update(Cmd const& cmd) {
    // Update sim time
    t += dt;
    // If the commanded timestep is more than numeric zero, then reassign the sim's timestep
    if(cmd.dt > EPS) {dt = cmd.dt;}
}

//////////////////////////////////////////////////

} // namespace physics
} // namespace stokesim
