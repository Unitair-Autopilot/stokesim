/** @file
Declaration of the Simulator class.
*/
#pragma once
#include <chrono>
#include <thread>
#include <dlfcn.h>
#include "stokesim/common.h"
#include "stokesim/io/plugin.h"
#include "stokesim/physics/world.h"

namespace stokesim {

//////////////////////////////////////////////////

/// Main class for a simulation. Manages real-time,
/// a World, and all Plugin coroutines.
class Simulator {
private:
    /// @name Provided
    //@{
    bool const        realtime; ///< Whether or not sim-time throttles back to real-time
    Escal const       end_time; ///< Terminal sim-time (negative implies endless sim)
    physics::World    world; ///< Top of a physics tree
    Pdict<io::Plugin> plugins; ///< The plugins that will work as coroutines during the sim
    //@}

    /// @name Derived
    //@{
    std::vector<Str>  plugin_names; ///< Keys for the plugins map
    //@}

public:
    /// @name Core
    //@{
    Simulator(YAML::Node const& config); ///< Construction given a YAML that defines all "Provided" members
    int simulate(); ///< Runs a simulation and returns an exit status integer upon completion (0 is nominal)
    int profile(); ///< Same as simulate() but also prints a bunch of performance information upon completion
    //@}

    /// @name Utilities
    //@{
    Escal query_realtime() const; ///< Returns the system Unix time in seconds
    void sleep_us(int dt) const; ///< Sleeps this thread for a given number of microseconds
    int test() const; ///< Runs a set of unit tests for the underlying physics
    //@}

    /// @name Access
    //@{
    inline bool                    get_realtime()              const {return realtime;}
    inline Escal                   get_end_time()              const {return end_time;}
    inline physics::World   const& get_world()                 const {return world;}
    inline io::Plugin       const& get_plugin(Str const& name) const {return *plugins.at(name);}
    inline std::vector<Str>        get_plugin_names()          const {return plugin_names;}
    //@}

    NO_COPY(Simulator)
};

//////////////////////////////////////////////////

} // namespace stokesim

/// @brief (for using Simulator from a library) @name Factory Functions
//@{
extern "C" {
    stokesim::Simulator* create_simulator(char const config_file[]); ///< Dynamic allocation of Simulator
    void destroy_simulator(stokesim::Simulator* ptr); ///< Release dynamically allocated Simulator memory
}
//@}
