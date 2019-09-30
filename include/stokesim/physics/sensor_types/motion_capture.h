/** @file
Declaration of the MotionCapture class.
*/
#pragma once
#include "stokesim/physics/sensor.h"

namespace stokesim {
namespace physics {
class Craft;
namespace sensor_types {

//////////////////////////////////////////////////

/// Simplified visual motion-capture sensor output using white and Ornstein noise to
/// act like the much more complicated uncertainty inherent to visual motion-capture,
/// which produces a pose estimate via some black-boxed estimation algorithm.
class MotionCapture : public Sensor {
public:
    /// @name Provided
    //@{
    Evec3 const pos; ///< Tracked point on the body in COM-coordinates
    Equat const ori; ///< Orientation offset relative to COM-coordinates
    Eigen::Matrix<Escal, 6, 6> const noise_covar; ///< Noise covariance matrix in ENU (DOFs: translation then orientation)
    Eigen::Matrix<Escal, 6, 1> const bias0; ///< Initial noise mean in ENU (DOFs: translation then orientation)
    Eigen::Matrix<Escal, 6, 1> const bias_taus; ///< Time-constants measuring the bias0-return tendency of bias (DOFs: translation then orientation)
    Eigen::Matrix<Escal, 6, 6> const bias_covar; ///< Covariance matrix in ENU for bias random walk (DOFs: translation then orientation)
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
    MotionCapture(YAML::Node const& config, Craft const* container);
    void update_states(Cmd const& cmd, Escal dt) override;
    void update_measurements(Cmd const& cmd) override;
    //@}

    NO_COPY(MotionCapture)
};

//////////////////////////////////////////////////

} // namespace sensor_types
} // namespace physics
} // namespace stokesim
