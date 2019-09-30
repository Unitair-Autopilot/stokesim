/** @file
Implementation of gps.h
*/
#include "stokesim/physics/sensor_types/gps.h"
#include "stokesim/physics/world.h"

namespace stokesim {
namespace physics {
namespace sensor_types {

//////////////////////////////////////////////////

GPS::GPS(YAML::Node const& config, Craft const* container) :
    Sensor(config, container),
    pos(evec_from_yaml(yget("pos", config), 3)),
    noise_covar(emat_from_yaml(yget("noise_covar", config), 6, 6)),
    bias0(evec_from_yaml(yget("bias0", config), 6)),
    bias_taus(evec_from_yaml(yget("bias_taus", config), 6).cwiseMax(craft->world->clock.dt)),
    bias_covar(emat_from_yaml(yget("bias_covar", config), 6, 6)),
    noise(Eigen::Matrix<Escal, 6, 1>::Zero()),
    noise_pdf(noise, noise_covar),
    bias(bias0),
    bias_springs(bias_taus.cwiseInverse()),
    bias_pdf(bias, bias_covar) {
    // Record valid measurement names
    measurement_names = {"lat", "lon", "alt", "vel_east", "vel_north", "vel_up", "groundspeed"};
    for(auto const& name : measurement_names) {measurements[name];}
}

//////////////////////////////////////////////////

void GPS::update_states(Cmd const& cmd, Escal dt) {
    // Sample noise and update bias Ornstein stochastic process
    noise = noise_pdf.samples(1);
    bias += sqrt(dt)*bias_pdf.samples(1) + dt*bias_springs.cwiseProduct(bias0-bias);
}

//////////////////////////////////////////////////

void GPS::update_measurements(Cmd const& cmd) {
    LLA const& lla = craft->body.lla;
    Evec3 noisy_lla_vec = Evec3(lla.lat, lla.lon, lla.alt) + noise.head<3>() + bias.head<3>();
    Evec3 noisy_vel_enu = craft->body.ori*craft->body.vel + noise.tail<3>() + bias.tail<3>();
    Evec3 noisy_vel_body = craft->body.vel + noise.tail<3>() + bias.tail<3>();
    measurements.at("lat") = noisy_lla_vec(0);
    measurements.at("lon") = noisy_lla_vec(1);
    measurements.at("alt") = noisy_lla_vec(2);
    measurements.at("vel_east") = noisy_vel_enu(0);
    measurements.at("vel_north") = noisy_vel_enu(1);
    measurements.at("vel_up") = noisy_vel_enu(2);
    measurements.at("groundspeed") = noisy_vel_body(0);
}

//////////////////////////////////////////////////

} // namespace sensor_types
} // namespace physics
} // namespace stokesim
