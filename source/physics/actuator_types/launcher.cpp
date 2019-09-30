/** @file
Implementation of launcher.h
*/
#include "stokesim/physics/actuator_types/launcher.h"
#include "stokesim/physics/world.h"

namespace stokesim {
namespace physics {
namespace actuator_types {

//////////////////////////////////////////////////

Launcher::Launcher(YAML::Node const& config, Craft const* container) :
    Actuator(config, container),
    timer0(yget("timer0", config).as<Escal>()),
    duration(yget("duration", config).as<Escal>()),
    tension(yget("tension", config).as<Escal>()),
    tension_loss_rate(yget("tension_loss_rate", config).as<Escal>()),
    vibe_covar(emat_from_yaml(yget("vibe_covar", config), 3, 3)),
    timer(timer0),
    status("waiting"),
    vibe_pdf(Evec3(0, 0, 0), vibe_covar) {
    // Record valid command value names
    value_names = {"timer"};
}

//////////////////////////////////////////////////

void Launcher::update_impact(Cmd const& cmd) {
    // If already launched, don't influence the craft
    if(status == "launched") {return;}
    // If new timer value was given, set it
    if(cmd.values.count("timer")) {timer = cmd.values.at("timer");}
    // Wrench needed to stabilize craft at the launcher pose
    Escal constexpr k = 150;
    force = k*(craft->body.ori.inverse()*(craft->body.pos0 - craft->body.pos)) - sqrt(k)*craft->body.vel;
    torque = k*math::qlogmap(craft->body.ori.inverse() * craft->body.ori0) - sqrt(k)*craft->body.avel;
    // Apply force depending on status
    if(status == "waiting") {
        // Switch to launching mode upon timer finish
        timer -= craft->world->clock.dt;
        if(timer < 0) {
            status = "launching";
            timer = duration;
        }
    } else if(status == "launching") {
        if(timer > 0) {
            force(0) = tension - math::clip(tension_loss_rate*(duration - timer), 0.0, INF);
            force += vibe_pdf.samples(1);
            timer -= craft->world->clock.dt;
        } else {
            status = "launched";
            timer = -1;
            force << 0, 0, 0;
            torque << 0, 0, 0;
        }
    }
}

//////////////////////////////////////////////////

} // namespace actuator_types
} // namespace physics
} // namespace stokesim
