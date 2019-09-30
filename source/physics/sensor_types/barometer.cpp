/** @file
Implementation of barometer.h
*/
#include "stokesim/physics/sensor_types/barometer.h"
#include "stokesim/physics/world.h"

namespace stokesim {
namespace physics {
namespace sensor_types {

//////////////////////////////////////////////////

Barometer::Barometer(YAML::Node const& config, Craft const* container) :
    Sensor(config, container),
    pos(evec_from_yaml(yget("pos", config), 3)),
    noise_stdev(yget("noise_stdev", config).as<Escal>()),
    bias(yget("bias", config).as<Escal>()),
    noise(0),
    noise_pdf(0, noise_stdev) {
    // Record valid measurement names
    measurement_names = {"pressure"};
    for(auto const& name : measurement_names) {measurements[name];}
}

//////////////////////////////////////////////////

void Barometer::update_states(Cmd const& cmd, Escal dt) {
    noise = noise_pdf(math::RNG);
}

//////////////////////////////////////////////////

void Barometer::update_measurements(Cmd const& cmd) {
    // Pressure of fluid at mounting point (variation matters a lot underwater), plus noise
    measurements.at("pressure") = craft->world->env.atmos.pressure_at_alt(craft->body.alt_of_point(pos)) + bias + noise;
}

//////////////////////////////////////////////////

} // namespace sensor_types
} // namespace physics
} // namespace stokesim
