/** @file
Declaration of the Env class.
*/
#pragma once
#include "stokesim/common.h"
#include "stokesim/physics/geoid.h"
#include "stokesim/physics/atmos.h"
#include "stokesim/physics/magneto.h"
#include "stokesim/physics/octree.h"

namespace stokesim {
namespace physics {
class World;

//////////////////////////////////////////////////

/// The environment of a World, containing its motion, fields, and obstacles.
class Env {
public:
    /// @name Provided
    //@{
    World const* world; ///< Pointer to the owner of this env
    Geoid        geoid;
    Atmos        atmos;
    Magneto      magneto;
    Octree       octree;
    //@}

    /// @name Types
    //@{
    struct Cmd {
        Geoid::Cmd geoid_cmd; ///< Command to pass on to this environment's geoid
        Atmos::Cmd atmos_cmd; ///< Command to pass on to this environment's atmosphere
        Magneto::Cmd magneto_cmd; ///< Command to pass on to this environment's magnetosphere
        Octree::Cmd octree_cmd; ///< Command to pass on to this environment's octree
        void merge(Cmd const& other);
    };
    //@}

private:
    /// @name Core
    //@{
    friend class World;
    Env(YAML::Node const& config, World const* container);
    void update(Cmd const& cmd);
    //@}

    NO_COPY(Env)
};

//////////////////////////////////////////////////

} // namespace physics
} // namespace stokesim
