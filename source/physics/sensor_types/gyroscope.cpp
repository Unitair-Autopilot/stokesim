/** @file
Implementation of gyroscope.h
*/
#include "stokesim/physics/sensor_types/gyroscope.h"
#include "stokesim/physics/world.h"

namespace stokesim {
namespace physics {
namespace sensor_types {

//////////////////////////////////////////////////

Gyroscope::Gyroscope(YAML::Node const& config, Craft const* container) :
    Sensor(config, container),
    pos(evec_from_yaml(yget("pos", config), 3)),
    rotmat(emat_from_yaml(yget("rotmat", config), 3, 3)),
    noise_covar(emat_from_yaml(yget("noise_covar", config), 3, 3)),
    bias0(evec_from_yaml(yget("bias0", config), 3)),
    bias_taus(evec_from_yaml(yget("bias_taus", config), 3).cwiseMax(craft->world->clock.dt)),
    bias_covar(emat_from_yaml(yget("bias_covar", config), 3, 3)),
    thermal_gain(yget("thermal_gain", config).as<Escal>()),
    rotmat_inv(rotmat.transpose()),
    noise(0, 0, 0),
    noise_pdf(Evec3(0, 0, 0), noise_covar),
    bias(bias0),
    bias_springs(bias_taus.cwiseInverse()),
    bias_pdf(Evec3(0, 0, 0), bias_covar),
    temperature(craft->world->env.atmos.T0) {
    // Record valid measurement names
    measurement_names = {"x", "y", "z"};
    for(auto const& name : measurement_names) {measurements[name];}
    state_names = {"bias_x", "bias_y", "bias_z", "temperature"};
    for(auto const& name : state_names) {states[name];}
}

//////////////////////////////////////////////////

void Gyroscope::update_states(Cmd const& cmd, Escal dt) {
    // Noise sample scaled by 1+k*temperature_deviation
    temperature = craft->world->env.atmos.temperature_at_alt(craft->body.lla.alt);
    noise = noise_pdf.samples(1) * (1 + thermal_gain*(temperature - craft->world->env.atmos.T0));
    // Update bias Ornstein stochastic process
    bias += sqrt(dt)*bias_pdf.samples(1) + dt*bias_springs.cwiseProduct(bias0-bias);
    // Expose the bias and temperature states
    states.at("bias_x") = bias(0);
    states.at("bias_y") = bias(1);
    states.at("bias_z") = bias(2);
    states.at("temperature") = temperature;
}

//////////////////////////////////////////////////

void Gyroscope::update_measurements(Cmd const& cmd) {
    // Gyroscope records inertial angular velocity expressed in sensor-coordinates
    Evec3 noisy_avel_inr = rotmat_inv*craft->body.avel_inr + noise + bias;
    measurements.at("x") = noisy_avel_inr(0);
    measurements.at("y") = noisy_avel_inr(1);
    measurements.at("z") = noisy_avel_inr(2);
}

//////////////////////////////////////////////////

} // namespace sensor_types
} // namespace physics
} // namespace stokesim
