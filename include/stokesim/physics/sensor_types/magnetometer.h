/** @file
Declaration of the Magnetometer class.
*/
#pragma once
#include "stokesim/physics/sensor.h"

namespace stokesim {
namespace physics {
class Craft;
namespace sensor_types {

//////////////////////////////////////////////////

/// Three-axis magnetometer subject to white noise, and
/// "hard/soft iron" distortions.
class Magnetometer : public Sensor {
public:
    /// @name Provided
    //@{
    Evec3 const pos; ///< Mounting position in COM-coordinates
    Emat3 const rotmat; ///< Mounting orientation in COM-coordinates
    Emat3 const noise_covar; ///< Noise covariance matrix
    Evec3 const hard_distort; ///< Hard-iron distortion vector in sensor-coordinates
    Emat3 const soft_distort; ///< Soft-iron distortion matrix in sensor-coordinates
    //@}

    /// @name Derived
    //@{
    Emat3 const    rotmat_inv; ///< Transpose of rotmat
    Evec3          noise; ///< Current value of noise
    math::Gaussian noise_pdf; ///< Noise probability density function
    //@}

private:
    /// @name Core
    //@{
    friend class stokesim::physics::Craft;
    Magnetometer(YAML::Node const& config, Craft const* container);
    void update_states(Cmd const& cmd, Escal dt) override;
    void update_measurements(Cmd const& cmd) override;
    //@}

    NO_COPY(Magnetometer)
};

//////////////////////////////////////////////////

} // namespace sensor_types
} // namespace physics
} // namespace stokesim
