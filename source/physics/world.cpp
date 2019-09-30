/** @file
Implementation of world.h
*/
#include "stokesim/physics/world.h"
#include "stokesim/simulator.h"

namespace stokesim {
namespace physics {

//////////////////////////////////////////////////

void World::Cmd::merge(Cmd const& other) {
    clock_cmd.merge(other.clock_cmd);
    env_cmd.merge(other.env_cmd);
    for(auto const& pair : other.craft_cmds) {craft_cmds[pair.first].merge(pair.second);}
}

//////////////////////////////////////////////////

World::World(YAML::Node const& config, Simulator const* container) :
    simulator(container),
    clock(yget("clock", config), this),
    env(yget("env", config), this) {
    // For each craft specification
    for(auto const& pair : yget("crafts", config)) {
        // Add craft name to the names list
        Str name = pair.first.as<Str>();
        craft_names.push_back(name);
        // Push a shared pointer to a new craft into the crafts map
        crafts.insert({name, std::shared_ptr<Craft>(new Craft(pair.second, this))});
    }
}

//////////////////////////////////////////////////

void World::update(Cmd const& cmd) {
    // For each craft
    for(auto & pair : crafts) {
        // If a command for that craft was given, update the craft with said command
        if(cmd.craft_cmds.count(pair.first)) {pair.second->update(cmd.craft_cmds.at(pair.first));}
        // Otherwise update the craft with a passive command
        else {pair.second->update(Craft::Cmd());}
    }
    // Update the environment and time
    env.update(cmd.env_cmd);
    clock.update(cmd.clock_cmd);
}

//////////////////////////////////////////////////

} // namespace physics
} // namespace stokesim
