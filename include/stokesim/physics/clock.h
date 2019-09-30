/** @file
Declaration of the Clock class.
*/
#pragma once
#include "stokesim/common.h"

namespace stokesim {
namespace physics {
class World;

//////////////////////////////////////////////////

/// Manages simulation time (not real time).
class Clock {
public:
    /// @name Provided
    //@{
    World const* world; ///< Pointer to the owner of this clock
    Escal const  dt0; ///< Initial integration timestep
    Escal const  date; ///< Decimal date (for time-dependent models), yrs (DEFAULT: 2020.0587 yrs)
    //@}

    /// @name Derived
    //@{
    Escal dt; ///< Current integration timestep
    Escal t; ///< Current sim time
    //@}

    /// @name Types
    //@{
    struct Cmd {
        Escal dt = -1; ///< New integration timestep (negative implies don't change)
        void merge(Cmd const& other);
    };
    //@}

private:
    /// @name Core
    //@{
    friend class World;
    Clock(YAML::Node const& config, World const* container);
    void update(Cmd const& cmd);
    //@}

    NO_COPY(Clock)
};

//////////////////////////////////////////////////

} // namespace physics
} // namespace stokesim
