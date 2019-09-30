/** @file
Implementation of the non-abstract parts of plugin.h
*/
#include "stokesim/io/plugin.h"

namespace stokesim {
namespace io {

//////////////////////////////////////////////////

void Plugin::Output::merge(Output const& other) {
    // Terminate latches to true
    if(other.terminate) {terminate = true;}
    // Descend into World::Cmd merge
    world_cmd.merge(other.world_cmd);
}

//////////////////////////////////////////////////

bool Plugin::configure_wrapper(YAML::Node const& config, physics::World const& world) {
    // Set provided attributes
    bin_file = yget("bin_file", config).as<Str>();
    call_freq = yget("call_freq", config).as<Escal>();
    // Safely calculate call period
    if(call_freq > EPS) {call_period = 1/call_freq;}
    // Descend into derived-class configure function
    return configure(config, world);
}

//////////////////////////////////////////////////

} // namespace io
} // namespace stokesim
