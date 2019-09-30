/** @file
Implementation of env.h
*/
#include "stokesim/physics/env.h"
#include "stokesim/physics/world.h"

namespace stokesim {
namespace physics {

//////////////////////////////////////////////////

void Env::Cmd::merge(Cmd const& other) {
    geoid_cmd.merge(other.geoid_cmd);
    atmos_cmd.merge(other.atmos_cmd);
    magneto_cmd.merge(other.magneto_cmd);
    octree_cmd.merge(other.octree_cmd);
}

//////////////////////////////////////////////////

Env::Env(YAML::Node const& config, World const* container) :
    world(container),
    geoid(yget("geoid", config), this),
    atmos(yget("atmos", config), this),
    magneto(yget("magneto", config), this),
    octree(yget("octree", config), this) {
}

//////////////////////////////////////////////////

void Env::update(Cmd const& cmd) {
    geoid.update(cmd.geoid_cmd);
    atmos.update(cmd.atmos_cmd);
    magneto.update(cmd.magneto_cmd);
    octree.update(cmd.octree_cmd);
}

//////////////////////////////////////////////////

} // namespace physics
} // namespace stokesim
