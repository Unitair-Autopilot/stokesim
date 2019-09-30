/** @file
Declaration of the RcChannels class.
*/
#pragma once
#include "stokesim/physics/sensor.h"

namespace stokesim {
namespace physics {
class Craft;
namespace sensor_types {

//////////////////////////////////////////////////

/// Remote-control channels for the radio on a single craft
class RcChannels : public Sensor {
public:
    /// @name Provided
    //@{
    int const num_channels; ///< The number of channels
    //@}

private:
    /// @name Core
    //@{
    friend class stokesim::physics::Craft;
    RcChannels(YAML::Node const& config, Craft const* container);
    void update_states(Cmd const& cmd, Escal dt) override;
    void update_measurements(Cmd const& cmd) override;
    //@}

    NO_COPY(RcChannels)
};

//////////////////////////////////////////////////

}  // namespace sensor_types
}  // namespace physics
}  // namespace stokesim
