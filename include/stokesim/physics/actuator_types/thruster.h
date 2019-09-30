/** @file
Declaration of the Thruster class.
*/
#pragma once
#include "stokesim/physics/actuator.h"

namespace stokesim {
namespace physics {
class Craft;
namespace actuator_types {

//////////////////////////////////////////////////

/// Electric-motor-driven propeller commanded by RPM. Has limits,
/// back-torque, dead-band, slew-rate, and energy consumption, but
/// an ideal ESC. Thrust-RPM model is quadratic.
class Thruster : public Actuator {
public:
    /// @name Provided
    //@{
    Escal const dia; ///< Diameter of the propeller
    Evec3 const pos; ///< Position vector in COM-coordinates of mounting point
    Evec3 const dir; ///< Direction vector in COM-coordinates of thrust at positive RPM
    Escal const c1; ///< Linear coefficient for thrust from RPM
    Escal const c2; ///< Quadratic coefficient for thrust from RPM
    Escal const kr; ///< Ratio of reaction torque to thrust (sign defines handedness)
    Escal const kv; ///< Thrust loss per unit squared velocity (>=0)
    Escal const ks; ///< Linear coefficient for slipstream wind from thrust (DEFAULT: 0)
    Escal const rpm_min; ///< Minimum RPM
    Escal const rpm_max; ///< Maximum RPM
    Escal const deadband; ///< Plus-or-minus RPM command dead-band (>=0)
    Escal const rpmdot; ///< Angular acceleration magnitude (>0)
    Escal const power_max; ///< Power consumption at maximum RPM (negative implies use torque*RPM)
    //@}

    /// @name Derived
    //@{
    Escal const area; ///< Total propeller area
    Escal rpm_cmd; ///< Current (latched) commanded RPM
    Escal rpm; ///< Current attained RPM
    //@}

    /// @name Utilities
    //@{
    Escal thrust_at_rpm(Escal rpm) const; ///< Computes the static thrust generated at the given RPM
    Escal rpm_at_thrust(Escal thrust) const; ///< Computes the RPM needed for a given static thrust
    //@}

private:
    /// @name Core
    //@{
    friend class stokesim::physics::Craft;
    Thruster(YAML::Node const& config, Craft const* container);
    void update_impact(Cmd const& cmd) override;
    void kill() override; ///< Sets rpm_cmd to zero
    void set_effort(Escal effort) override; ///< Effort is on [0, 1]
    //@}

    NO_COPY(Thruster)
};

//////////////////////////////////////////////////

} // namespace actuator_types
} // namespace physics
} // namespace stokesim
