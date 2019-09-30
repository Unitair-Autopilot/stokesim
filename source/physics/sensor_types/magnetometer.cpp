/** @file
Implementation of magnetometer.h
*/
#include "stokesim/physics/sensor_types/magnetometer.h"
#include "stokesim/physics/world.h"

namespace stokesim {
namespace physics {
namespace sensor_types {

//////////////////////////////////////////////////

Magnetometer::Magnetometer(YAML::Node const& config, Craft const* container) :
    Sensor(config, container),
    pos(evec_from_yaml(yget("pos", config), 3)),
    rotmat(emat_from_yaml(yget("rotmat", config), 3, 3)),
    noise_covar(emat_from_yaml(yget("noise_covar", config), 3, 3)),
    hard_distort(evec_from_yaml(yget("hard_distort", config), 3)),
    soft_distort(emat_from_yaml(yget("soft_distort", config), 3, 3)),
    rotmat_inv(rotmat.transpose()),
    noise(0, 0, 0),
    noise_pdf(Evec3(0, 0, 0), noise_covar) {
    // Record valid measurement names
    measurement_names = {"x", "y", "z"};
    for(auto const& name : measurement_names) {measurements[name];}
}

//////////////////////////////////////////////////

void Magnetometer::update_states(Cmd const& cmd, Escal dt) {
    noise = noise_pdf.samples(1);
}

//////////////////////////////////////////////////

void Magnetometer::update_measurements(Cmd const& cmd) {
    Env const& env = craft->world->env;
    Body const& body = craft->body;
    // Rotate bfield from ENU to sensor-coordinates
    Evec3 bfield = rotmat_inv * body.ori_inv * env.magneto.bfield_at_lla(body.lla);
    // Distort, add hard offset and noise
    Evec3 noisy_mag = soft_distort*bfield + hard_distort + noise;
    measurements.at("x") = noisy_mag(0);
    measurements.at("y") = noisy_mag(1);
    measurements.at("z") = noisy_mag(2);
}

//////////////////////////////////////////////////

} // namespace sensor_types
} // namespace physics
} // namespace stokesim
