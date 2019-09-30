/** @file
Declaration of the Octree class.
*/
#pragma once
#include "stokesim/common.h"

namespace stokesim {
namespace physics {
class Env;

//////////////////////////////////////////////////

/// (UNFINISHED) Space-partitioning data-structure for
/// keeping track of tangible objects in the environment.
class Octree {
public:
    /// @name Provided
    //@{
    Env const* env; ///< Pointer to the owner of this octree
    Escal const ground_alt; ///< Altitude of ground-level (DEFAULT: env->geoid.enu0_lla.alt)
    //@}

    /// @name Derived
    //@{
    //@}

    /// @name Utilities
    //@{
    bool is_collided(Evec3 const& p, Escal r) const; ///< (UNFINISHED)
    //@}

    /// @name Types
    //@{
    struct Cmd {
        void merge(Cmd const& other);
    };
    //@}

private:
    /// @name Core
    //@{
    friend class Env;
    Octree(YAML::Node const& config, Env const* container);
    void update(Cmd const& cmd);
    //@}

    NO_COPY(Octree)
};

//////////////////////////////////////////////////

} // namespace physics
} // namespace stokesim
