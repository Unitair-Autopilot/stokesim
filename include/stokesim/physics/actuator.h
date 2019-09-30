/** @file
Declaration of the Actuator class.
*/
#pragma once
#include "stokesim/common.h"
#include "stokesim/math/so3.h"

namespace stokesim {
namespace physics {
class Craft;

//////////////////////////////////////////////////

/// Base class for an actuator: something that can exert force
/// and torque on a Craft's Body with controllable efforts.
class Actuator {
public:
    /// @name Core
    //@{
    Actuator(YAML::Node const& config, Craft const* container);
    //@}

    /// @name Provided
    //@{
    Craft const* craft; ///< Pointer to the owner of this actuator
    Str const    type; ///< Name of the derived class implementing this actuator
    //@}

    /// @name Derived
    //@{
    Evec3 force; ///< Force exerted on the body in COM-coordinates
    Evec3 torque; ///< Torque exerted on the body in COM-coordinates
    Escal power; ///< Power instantaneously consumed
    Evec3 wind; ///< Slipstream wind generated over the body in COM-coordinates
    bool killed; ///< Whether or not this actuator has been killed by a trigger, other plugin, etc
    std::vector<Str> value_names; ///< List of the valid keys for the Cmd::values map
    //@}

    /// @name Types
    //@{
    struct Cmd {
        Dict<Escal> values; ///< Dictionary of actuation command values
        Opt<bool> kill; ///< Set (true) or unset (false) the kill state of this actuator
        void merge(Cmd const& other);
    };
    //@}

protected:
    /// @name Core
    //@{
    friend class Craft;
    virtual void update(Cmd const& cmd);
    virtual void update_impact(Cmd const& cmd) =0; ///< Assigns force, torque, and power
    virtual void kill(){}; ///< Makes appropriate changes for being killed / zero battery
    virtual void set_effort(Escal effort){}; ///< Sets whatever makes sense for a normalized "effort" command
    //@}

    NO_COPY(Actuator)
};

//////////////////////////////////////////////////

} // namespace physics
} // namespace stokesim
