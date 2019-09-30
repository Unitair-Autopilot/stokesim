/** @file
Implementation of thruster.h
*/
#include "stokesim/physics/actuator_types/thruster.h"
#include "stokesim/physics/world.h"

namespace stokesim {
namespace physics {
namespace actuator_types {

//////////////////////////////////////////////////

Escal Thruster::thrust_at_rpm(Escal rpm) const {
    // Quadratic relationship with zero offset
    return c1*rpm + c2*fabs(rpm)*rpm;
}

//////////////////////////////////////////////////

Escal Thruster::rpm_at_thrust(Escal thrust) const {
    // Inverse of thrust_at_rpm function
    if(fabs(c2) < EPS) return thrust/c1;
    if(thrust > 0) return -(c1 - sqrt(pow(c1, 2) + 4*c2*thrust)) / (2*c2);
    return (c1 - sqrt(pow(c1, 2) - 4*c2*thrust)) / (2*c2);
}

//////////////////////////////////////////////////

Thruster::Thruster(YAML::Node const& config, Craft const* container) :
    Actuator(config, container),
    dia(yget("dia", config).as<Escal>()),
    pos(evec_from_yaml(yget("pos", config), 3)),
    dir(evec_from_yaml(yget("dir", config), 3).normalized()),
    c1(yget("c1", config).as<Escal>()),
    c2(yget("c2", config).as<Escal>()),
    kr(yget("kr", config).as<Escal>()),
    kv(fabs(yget("kv", config).as<Escal>())),
    ks(yget("ks", config, 0).as<Escal>()),
    rpm_min(yget("rpm_min", config).as<Escal>()),
    rpm_max(yget("rpm_max", config).as<Escal>()),
    deadband(fabs(yget("deadband", config).as<Escal>())),
    rpmdot(fabs(yget("rpmdot", config).as<Escal>())),
    power_max(yget("power_max", config).as<Escal>()),
    area(PI*dia*dia/4),
    rpm_cmd(0),
    rpm(0) {
    // Record valid command value names
    value_names = {"rpm"};
}

//////////////////////////////////////////////////

void Thruster::update_impact(Cmd const& cmd) {
    // If RPM command was given, latch it
    if(cmd.values.count("rpm")) {rpm_cmd = cmd.values.at("rpm");}
    else if(cmd.values.count("effort")) {set_effort(cmd.values.at("effort"));}
    // Compute ESC error
    Escal err = rpm_cmd - rpm;
    // Largest change in RPM possible this timestep
    Escal drpm = rpmdot * craft->world->clock.dt;
    // Make exact change to current RPM according to sign of error (this is ideal ESC response)
    if(fabs(err) < drpm) rpm = rpm_cmd;
    else if(err > 0) rpm += drpm;
    else rpm -= drpm;
    // Enforce actuator saturation
    rpm = math::clip(rpm, rpm_min, rpm_max);
    // Find airspeed at mounting point in direction of actuator (excluding effect of self-generated slipstream)
    Escal airspeed = (craft->body.vel_rel + wind + craft->body.avel.cross(pos)).dot(dir);
    // Compute static thrust for the current RPM
    Escal thr0 = thrust_at_rpm(rpm);
    // Decay thrust according to airspeed
    Escal thr = thr0*math::clip(1-kv*fabs(airspeed)*airspeed*math::signum(thr0), 0.0, 1.0);
    // Update force and torque vectors
    force = thr*dir;
    torque = kr*force + pos.cross(force);
    // Update self-generated slipstream proportional to thrust force
    wind = -ks*force;
    // Update instantaneous power consumption as linear function of RPM...
    if(power_max >= 0) {power = fabs(rpm/rpm_max) * power_max;}
    // ... or as motor torque * RPM if specified by negative power_max
    else {power = fabs((kr*thr)*(rpm*PI/30));}
}

//////////////////////////////////////////////////

void Thruster::kill() {
    // Latched RPM command is set to zero (effectively killing the ideal ESC)
    rpm_cmd = 0;
}

//////////////////////////////////////////////////

void Thruster::set_effort(Escal effort) {
    // ESC RPM command is set to rpm_minimum + effort*rpm_range where effort is on [0, 1]
    rpm_cmd = rpm_min + effort*(rpm_max-rpm_min);
}

//////////////////////////////////////////////////

} // namespace actuator_types
} // namespace physics
} // namespace stokesim
