/** @file
Implementation of motion_capture.h
*/
#include "stokesim/physics/sensor_types/motion_capture.h"
#include "stokesim/physics/world.h"

namespace stokesim {
namespace physics {
namespace sensor_types {

//////////////////////////////////////////////////

MotionCapture::MotionCapture(YAML::Node const& config, Craft const* container) :
    Sensor(config, container),
    pos(evec_from_yaml(yget("pos", config), 3)),
    ori(equat_from_yaml(yget("ori", config))),
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
    measurement_names = {"pos_x", "pos_y", "pos_z", "ori_w", "ori_x", "ori_y", "ori_z"};
    for(auto const& name : measurement_names) {measurements[name];}
}

//////////////////////////////////////////////////

void MotionCapture::update_states(Cmd const& cmd, Escal dt) {
    // Sample noise and update bias Ornstein stochastic process
    noise = noise_pdf.samples(1);
    bias += sqrt(dt)*bias_pdf.samples(1) + dt*bias_springs.cwiseProduct(bias0-bias);
}

//////////////////////////////////////////////////

void MotionCapture::update_measurements(Cmd const& cmd) {
    // Compute the tracked position and orientation in ENU, perturbed by the noise and bias
    Evec3 noisy_pos_enu = noise.head<3>() + bias.head<3>() + craft->body.pos + craft->body.ori*pos;
    Equat noisy_ori_enu = math::qexpmap(noise.tail<3>() + bias.tail<3>()) * craft->body.ori * ori;
    // Record the measurement
    measurements.at("pos_x") = noisy_pos_enu(0);
    measurements.at("pos_y") = noisy_pos_enu(1);
    measurements.at("pos_z") = noisy_pos_enu(2);
    measurements.at("ori_w") = noisy_ori_enu.w();
    measurements.at("ori_x") = noisy_ori_enu.x();
    measurements.at("ori_y") = noisy_ori_enu.y();
    measurements.at("ori_z") = noisy_ori_enu.z();
}

//////////////////////////////////////////////////

} // namespace sensor_types
} // namespace physics
} // namespace stokesim
