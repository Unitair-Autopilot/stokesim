/** @file
Implementation of surface.h
*/
#include "stokesim/physics/actuator_types/surface.h"
#include "stokesim/physics/world.h"

namespace stokesim {
namespace physics {
namespace actuator_types {

//////////////////////////////////////////////////

Surface::Surface(YAML::Node const& config, Craft const* container) :
    Actuator(config, container),
    dims(yget("dims", config).as<std::vector<Escal>>()),
    coeff(math::clip(yget("coeff", config).as<Escal>(), 0.0, INF)),
    origin(evec_from_yaml(yget("origin", config), 3)),
    orient0(equat_from_yaml(yget("orient0", config))),
    cenpres(evec_from_yaml(yget("cenpres", config), 3)),
    axis(evec_from_yaml(yget("axis", config), 3).normalized()),
    ang_min(yget("ang_min", config).as<Escal>()),
    ang_max(yget("ang_max", config).as<Escal>()),
    angdot(fabs(yget("angdot", config).as<Escal>())),
    flutter_amp(yget("flutter_amp", config, 0).as<Escal>()),
    flutter_freq(yget("flutter_freq", config, 0).as<Escal>()),
    area(fabs(dims[0]*dims[1])),
    normal(0, 0, 1),
    ang_cmd(0),
    ang(0),
    orient(orient0) {
    // Record valid command value names
    value_names = {"ang"};
}

//////////////////////////////////////////////////

void Surface::update_impact(Cmd const& cmd) {
    // If deflection angle command was given, latch it
    if(cmd.values.count("ang")) {ang_cmd = cmd.values.at("ang");}
    else if(cmd.values.count("effort")) {set_effort(cmd.values.at("effort"));}
    // Compute servo error
    Escal err = math::amod(ang_cmd - ang, true);
    // Largest change in angle possible this timestep
    Escal dang = angdot * craft->world->clock.dt;
    // Make exact change to current angle according to sign of error (this is ideal servo response)
    if(fabs(err) < dang) ang = ang_cmd;
    else if(err > 0) ang += dang;
    else ang -= dang;
    // Enforce actuator saturation
    ang = math::amod(math::clip(ang, ang_min, ang_max), true);
    // Compute the current wing deflection due to flutter
    Escal flutter = flutter_amp * sin(2*PI*flutter_freq*(craft->world->clock.t)) * craft->body.vel_rel.norm();
    // Compute surface-coordinates orientation from servo angle and flutter deflection
    orient = orient0*math::qexpmap(DEG2RAD*(ang + flutter)*axis);
    // Position of surface center of pressure rotated onto COM-coordinates
    Evec3 pos_cp = origin + orient*cenpres;
    // Velocity of the point pos_cp on the body, converted to surface-coordinates
    Evec3 vel_surf = orient.inverse()*(craft->body.vel_rel + craft->body.avel.cross(pos_cp));
    // Density at pos_cp
    Escal density = craft->world->env.atmos.density_at_alt(craft->body.alt_of_point(pos_cp));
    // Drag calculation carried out in surface-coordinates and then converted to COM-coordinates to update force and torque
    force = orient*(-density*area*coeff*fabs(vel_surf(2))*vel_surf(2)*normal);
    torque = pos_cp.cross(force);
}

//////////////////////////////////////////////////

void Surface::set_effort(Escal effort) {
    // Servo angle command is set to deflection_center + effort*deflection_range/2 where effort is on [-1, 1]
    ang_cmd = ((ang_max+ang_min) + effort*(ang_max-ang_min)) / 2;
}

//////////////////////////////////////////////////

} // namespace actuator_types
} // namespace physics
} // namespace stokesim
