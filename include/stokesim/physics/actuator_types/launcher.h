/** @file
Declaration of the Launcher class.
*/
#pragma once
#include "stokesim/physics/actuator.h"
#include "stokesim/math/gaussian.h"

namespace stokesim {
namespace physics {
class Craft;
namespace actuator_types {

//////////////////////////////////////////////////

/// A launcher rail that holds a craft in place and eventually shoots it off.
class Launcher : public Actuator {
public:
    /// @name Provided
    //@{
    Escal const timer0; ///< Initial value of the amount of time left till launch
    Escal const duration; ///< The amount of time through which the launcher force is exerted
    Escal const tension; ///< The forward force that will be exerted on the craft upon launch
    Escal const tension_loss_rate; ///< The amount of tension lost per unit time of launching
    Emat3 const vibe_covar; ///< Covariance matrix for vibrational noise during launching
    //@}

    /// @name Derived
    //@{
    Escal timer; ///< The current time left till launch (commandable)
    Str status; ///< "waiting", "launching", or "launched"
    math::Gaussian vibe_pdf; ///< Probability density function associated with vibe_covar
    //@}

private:
    /// @name Core
    //@{
    friend class stokesim::physics::Craft;
    Launcher(YAML::Node const& config, Craft const* container);
    void update_impact(Cmd const& cmd) override;
    //@}

    NO_COPY(Launcher)
};

//////////////////////////////////////////////////

} // namespace actuator_types
} // namespace physics
} // namespace stokesim
