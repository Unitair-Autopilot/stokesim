/** @file
Declaration of the Barometer class.
*/
#pragma once
#include "stokesim/physics/sensor.h"

namespace stokesim {
namespace physics {
class Craft;
namespace sensor_types {

//////////////////////////////////////////////////

/// Absolute barometer subject to white noise and bias.
class Barometer : public Sensor {
public:
    /// @name Provided
    //@{
    Evec3 const pos; ///< Mounting position in COM-coordinates
    Escal const noise_stdev; ///< Standard deviation of noise
    Escal const bias; ///< Mean of noise
    //@}

    /// @name Derived
    //@{
    Escal noise; ///< Current value of noise
    std::normal_distribution<Escal> noise_pdf; ///< Noise probability density function
    //@}

private:
    /// @name Core
    //@{
    friend class stokesim::physics::Craft;
    Barometer(YAML::Node const& config, Craft const* container);
    void update_states(Cmd const& cmd, Escal dt) override;
    void update_measurements(Cmd const& cmd) override;
    //@}

    NO_COPY(Barometer)
};

//////////////////////////////////////////////////

} // namespace sensor_types
} // namespace physics
} // namespace stokesim
