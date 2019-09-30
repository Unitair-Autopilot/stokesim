/** @file
Declaration of the GPS class.
*/
#pragma once
#include "stokesim/physics/sensor.h"

namespace stokesim {
namespace physics {
class Craft;
namespace sensor_types {

//////////////////////////////////////////////////

/// Simplified GPS sensor output using white and Ornstein noise to
/// act like the much more complicated uncertainty inherent to GPS.
class GPS : public Sensor {
public:
    /// @name Provided
    //@{
    Evec3 const pos; ///< Mounting position (of antenna) in COM-coordinates (currently unused in calculation)
    Eigen::Matrix<Escal, 6, 6> const noise_covar; ///< Noise covariance matrix (DOFs: lat, lon, alt, vel_east, vel_north, vel_up)
    Eigen::Matrix<Escal, 6, 1> const bias0; ///< Initial noise mean (DOFs: lat, lon, alt, vel_east, vel_north, vel_up)
    Eigen::Matrix<Escal, 6, 1> const bias_taus; ///< Time-constants measuring the bias0-return tendency of bias
    Eigen::Matrix<Escal, 6, 6> const bias_covar; ///< Covariance matrix for bias random walk
    //@}

    /// @name Derived
    //@{
    Eigen::Matrix<Escal, 6, 1> noise; ///< Current value of noise
    math::Gaussian noise_pdf; ///< Noise probability density function
    Eigen::Matrix<Escal, 6, 1> bias; ///< Current value of noise mean
    Eigen::Matrix<Escal, 6, 1> const bias_springs; ///< Reciprocals of bias_taus
    math::Gaussian bias_pdf; ///< Bias perturbation probability density function
    //@}

private:
    /// @name Core
    //@{
    friend class stokesim::physics::Craft;
    GPS(YAML::Node const& config, Craft const* container);
    void update_states(Cmd const& cmd, Escal dt) override;
    void update_measurements(Cmd const& cmd) override;
    //@}

    NO_COPY(GPS)
};

//////////////////////////////////////////////////

} // namespace sensor_types
} // namespace physics
} // namespace stokesim
