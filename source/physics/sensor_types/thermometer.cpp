/** @file
Implementation of thermometer.h
*/
#include "stokesim/physics/sensor_types/thermometer.h"
#include "stokesim/physics/world.h"

namespace stokesim {
namespace physics {
namespace sensor_types {

//////////////////////////////////////////////////

Thermometer::Thermometer(YAML::Node const& config, Craft const* container) :
    Sensor(config, container),
    pos(evec_from_yaml(yget("pos", config), 3)),
    noise_stdev(yget("noise_stdev", config).as<Escal>()),
    bias(yget("bias", config).as<Escal>()),
    noise(0),
    noise_pdf(0, noise_stdev) {
    // Record valid measurement names
    measurement_names = {"temperature"};
    for(auto const& name : measurement_names) {measurements[name];}
}

//////////////////////////////////////////////////

void Thermometer::update_states(Cmd const& cmd, Escal dt) {
    noise = noise_pdf(math::RNG);
}

//////////////////////////////////////////////////

void Thermometer::update_measurements(Cmd const& cmd) {
    measurements.at("temperature") = craft->world->env.atmos.temperature_at_alt(craft->body.lla.alt) + noise;
}

//////////////////////////////////////////////////

} // namespace sensor_types
} // namespace physics
} // namespace stokesim
