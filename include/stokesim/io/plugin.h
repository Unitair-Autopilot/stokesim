/** @file
Abstract base class that must be implemented by all plugins.
In addition, all plugins MUST provide create() and destroy()
C-type "factory" functions for their derived class. Their
creation is made easy with the EXPORT_AS_PLUGIN macro function.
*/
#pragma once
#include "stokesim/common.h"
#include "stokesim/physics/world.h"

namespace stokesim {
namespace io {

//////////////////////////////////////////////////

/// Abstract base class that must be implemented by all plugins
class Plugin {
private:
    /// @name Provided
    //@{
    Str   bin_file;  ///< Path to the shared-object (DLL) that implements this plugin
    Escal call_freq; ///< Sim-time frequency (Hz) at which this plugin's work function is called
    //@}

public:
    /// @name Derived
    //@{
    Escal last_call   = -INF; ///< Last sim-time that this plugin's work function was called
    Escal call_period =  INF; ///< Inverse of call_freq
    //@}

    /// @name Types
    //@{
    /// Container of results that the work function can produce
    struct Output {
        bool terminate = false; ///< Signal to cleanly terminate the whole simulation asap
        physics::World::Cmd world_cmd; ///< Physics command tree
        void merge(Output const& other); ///< Properly combines two plugin's outputs according to the merge rules of the physics command tree
    };
    //@}

    /// @name Abstract Framework
    //@{
    virtual ~Plugin() {} ///< Empty destructor can be overridden if desired
    virtual bool configure(YAML::Node const& config, physics::World const& world) =0; ///< Configuration function called once at the beginning of the simulation
    virtual Output work(physics::World const& world) =0; ///< Coroutine function called every call_period simulation seconds
    //@}

    /// @name Instantiation Tools
    //@{
    Plugin() =default;
    bool configure_wrapper(YAML::Node const& config, physics::World const& world); ///< Sets the base attributes and then calls configure()
    typedef Plugin* Creator(void);
    typedef void Destroyer(Plugin*);
    //@}
};

//////////////////////////////////////////////////

} // namespace io
} // namespace stokesim

/// In a derived class's implementation file, use this macro function to generate factory functions
#define EXPORT_AS_PLUGIN(CLASS) \
extern "C" {\
    CLASS* create(void) {\
        return new CLASS;\
    }\
    void destroy(CLASS* ptr) {\
        delete ptr;\
    }\
}
