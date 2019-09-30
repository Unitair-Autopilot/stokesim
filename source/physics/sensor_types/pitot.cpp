/** @file
Implementation of pitot.h
*/
#include "stokesim/physics/sensor_types/pitot.h"
#include "stokesim/physics/world.h"

namespace stokesim {
namespace physics {
namespace sensor_types {

//////////////////////////////////////////////////

Pitot::Pitot(YAML::Node const& config, Craft const* container) :
    Sensor(config, container),
    pos(evec_from_yaml(yget("pos", config), 3)),
    dir(evec_from_yaml(yget("dir", config), 3).normalized()),
    noise_stdev(yget("noise_stdev", config).as<Escal>()),
    bias(yget("bias", config).as<Escal>()),
    noise(0),
    noise_pdf(0, noise_stdev) {
    // Record valid measurement names
    measurement_names = {"airspeed", "differential_pressure"};
    for(auto const& name : measurement_names) {measurements[name];}
}

//////////////////////////////////////////////////

void Pitot::update_states(Cmd const& cmd, Escal dt) {
    noise = noise_pdf(math::RNG);
}

//////////////////////////////////////////////////

void Pitot::update_measurements(Cmd const& cmd) {
    // Velocity at mounting point decomposed onto sensing direction
    Escal airspeed = (craft->body.vel_rel + craft->body.avel.cross(pos)).dot(dir);
    // Density of fluid at mounting point and differential pressure
    Escal density = craft->world->env.atmos.density_at_alt(craft->body.alt_of_point(pos));
    Escal noisy_differential_pressure = math::clip(0.5*density*airspeed*airspeed + noise, 0.0, INF);
    measurements.at("differential_pressure") = noisy_differential_pressure;
    // Sometimes these sensors also provide a rough airspeed calculation from differential pressure
    measurements.at("airspeed") = sqrt(2*noisy_differential_pressure/density);
}

//////////////////////////////////////////////////

} // namespace sensor_types
} // namespace physics
} // namespace stokesim
