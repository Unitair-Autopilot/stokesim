/** @file
Implementation of octree.h
*/
#include "stokesim/physics/octree.h"
#include "stokesim/physics/world.h"

namespace stokesim {
namespace physics {

//////////////////////////////////////////////////

bool Octree::is_collided(Evec3 const& p, Escal r) const { // (UNFINISHED)
    return false;
}

//////////////////////////////////////////////////

void Octree::Cmd::merge(Cmd const& other) {
}

//////////////////////////////////////////////////

Octree::Octree(YAML::Node const& config, Env const* container) :
    env(container),
    ground_alt(yget("ground_alt", config, env->geoid.enu0_lla.alt).as<Escal>()) {
}

//////////////////////////////////////////////////

void Octree::update(Cmd const& cmd) { // (UNFINISHED)
}

//////////////////////////////////////////////////

} // namespace physics
} // namespace stokesim
