/** @file
Declaration of the World class.
*/
#pragma once
#include "stokesim/common.h"
#include "stokesim/physics/clock.h"
#include "stokesim/physics/env.h"
#include "stokesim/physics/craft.h"

namespace stokesim {
class Simulator;
namespace physics {

//////////////////////////////////////////////////

/// Top level for the simulation's tree of physics entities.
class World {
private:
    /// @name Provided
    //@{
    Pdict<Craft>     crafts; ///< Map from craft names to shared pointers at each craft

public:
    Simulator const* simulator; ///< Pointer to the owner of this world
    Clock            clock;
    Env              env;
    //@}

    /// @name Derived
    //@{
    std::vector<Str> craft_names; ///< List of all the keys for the crafts map
    //@}

    /// @name Types
    //@{
    struct Cmd {
        Clock::Cmd clock_cmd; ///< Command to pass on to this world's clock
        Env::Cmd env_cmd; ///< Command to pass on to this world's environment
        Dict<Craft::Cmd> craft_cmds; ///< Commands to pass on to this world's crafts
        void merge(Cmd const& other);
    };
    //@}

    /// @name Access
    //@{
    inline Craft const& get_craft(Str const& name) const {return *crafts.at(name);}
    inline bool         has_craft(Str const& name) const {return crafts.count(name);}
    //@}

private:
    /// @name Core
    //@{
    friend class stokesim::Simulator;
    World(YAML::Node const& config, Simulator const* container);
    void update(Cmd const& cmd);
    //@}

    NO_COPY(World)
};

//////////////////////////////////////////////////

} // namespace physics
} // namespace stokesim
