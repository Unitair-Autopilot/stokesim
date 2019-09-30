/** @file
Declaration of the Gyroscope class.
*/
#pragma once
#include "stokesim/physics/sensor.h"

namespace stokesim {
namespace physics {
class Craft;
namespace sensor_types {
class IMU;

//////////////////////////////////////////////////

/// Three-axis MEMS gyroscope subject to white and
/// Ornstein noise, and thermal variation.
class Gyroscope : public Sensor {
public:
    /// @name Provided
    //@{
    Evec3 const pos; ///< Mounting position in COM-coordinates
    Emat3 const rotmat; ///< Mounting orientation in COM-coordinates
    Emat3 const noise_covar; ///< Noise covariance matrix
    Evec3 const bias0; ///< Initial noise mean
    Evec3 const bias_taus; ///< Time-constants measuring the bias0-return tendency of bias
    Emat3 const bias_covar; ///< Covariance matrix for bias random walk
    Escal const thermal_gain; ///< Scale factor for temperature deviation's effect on noise
    //@}

    /// @name Derived
    //@{
    Emat3 const    rotmat_inv; ///< Transpose of rotmat
    Evec3          noise; ///< Current value of noise
    math::Gaussian noise_pdf; ///< Noise probability density function
    Evec3          bias; ///< Current value of noise mean
    Evec3 const    bias_springs; ///< Reciprocals of bias_taus
    math::Gaussian bias_pdf; ///< Bias perturbation probability density function
    Escal          temperature; ///< Thermal temperature
    //@}

private:
    /// @name Core
    //@{
    friend class stokesim::physics::Craft;
    friend class stokesim::physics::sensor_types::IMU;
    Gyroscope(YAML::Node const& config, Craft const* container);
    void update_states(Cmd const& cmd, Escal dt) override;
    void update_measurements(Cmd const& cmd) override;
    //@}

    NO_COPY(Gyroscope)
};

//////////////////////////////////////////////////

} // namespace sensor_types
} // namespace physics
} // namespace stokesim
