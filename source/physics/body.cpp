/** @file
Implementation of body.h
*/
#include "stokesim/physics/body.h"
#include "stokesim/physics/world.h"

namespace stokesim {
namespace physics {

//////////////////////////////////////////////////

Euler Body::compute_euler(bool use_degrees/*=true*/) const {
    return math::euler_from_quat(ori, use_degrees);
}

//////////////////////////////////////////////////

Escal Body::compute_ang_attack(bool use_degrees/*=true*/) const {
    Escal ang = atan2(-vel_rel(2), vel_rel(0));
    if(use_degrees) {return RAD2DEG*ang;}
    return ang;
}

//////////////////////////////////////////////////

Escal Body::compute_ang_sideslip(bool use_degrees/*=true*/) const {
    Escal ang = asin(-vel_rel(1) / vel_rel.norm());
    if(use_degrees) {return RAD2DEG*ang;}
    return ang;
}

//////////////////////////////////////////////////

Escal Body::alt_of_point(Evec3 const& p) const {
    // Add component of lever-arm along local up direction to COM altitude
    return lla.alt + (ori*p).dot(local_up);
}

//////////////////////////////////////////////////

Body::Body(YAML::Node const& config, Craft const* container) :
    craft(container),
    pos0(evec_from_yaml(yget("pos0", config), 3)),
    ori0(equat_from_yaml(yget("ori0", config))),
    vel0(evec_from_yaml(yget("vel0", config), 3)),
    avel0(evec_from_yaml(yget("avel0", config), 3)),
    mass(yget("mass", config).as<Escal>()),
    inertia(emat_from_yaml(yget("inertia", config), 3, 3)),
    centroid(evec_from_yaml(yget("centroid", config), 3)),
    bounding_box(evec_from_yaml(yget("bounding_box", config), 3)),
    areas(evec_from_yaml(yget("areas", config), 3)),
    volume(yget("volume", config).as<Escal>()),
    cenpres(evec_from_yaml(yget("cenpres", config), 3)),
    Cp1(emat_from_yaml(yget("Cp1", config), 3, 3)),
    Cp2(emat_from_yaml(yget("Cp2", config), 3, 3)),
    Cq(emat_from_yaml(yget("Cq", config), 3, 3)),
    bounciness(math::clip(yget("bounciness", config, 0.001).as<Escal>(), 0.0, 1.0)),
    lla(craft->world->env.geoid.lla_from_enu(pos0)),
    local_up(0, 0, 1),
    pos_ecef(craft->world->env.geoid.ecef_from_enu(pos0)),
    pos(pos0),
    ori(ori0),
    ori_inv(ori0.inverse()),
    vel(vel0),
    vel_rel(vel0),
    avel(avel0),
    avel_inr(avel0),
    veldot(0, 0, 0),
    veldot_inr(0, 0, 0),
    aveldot(0, 0, 0),
    aveldot_inr(0, 0, 0),
    inertia_inv(inertia.inverse()) {
}

//////////////////////////////////////////////////

void Body::update(Cmd const& cmd) {
    // Abbreviations
    Geoid const& geoid = craft->world->env.geoid;
    Atmos const& atmos = craft->world->env.atmos;
    Escal dt = craft->world->clock.dt;

    // Environmental conditions at center-of-mass in ENU coordinates
    Evec3 gravity_env = geoid.ecef_enu * geoid.gravity_at_ecef(pos_ecef);
    Evec3 wind_env = geoid.ecef_enu * atmos.wind_at_lla(lla);
    Escal density_env = atmos.density_at_alt(lla.alt);

    // Buoyancy
    Evec3 force_buoy = ori_inv * (-density_env * volume * gravity_env);
    Evec3 torque_buoy = centroid.cross(force_buoy);

    // Actuators
    Evec3 force_act = Evec3::Zero();
    Evec3 torque_act = Evec3::Zero();
    Evec3 wind_act = Evec3::Zero();
    for(auto const& name : craft->actuator_names) {
        Actuator const& actuator = craft->get_actuator(name);
        force_act += actuator.force;
        torque_act += actuator.torque;
        wind_act += actuator.wind;
    }

    // Drag
    Evec3 vel_cp = vel_rel + avel.cross(cenpres);
    Evec3 force_drag = -density_env*areas.cwiseProduct(Cp1*vel_cp + Cp2*(vel_cp.cwiseAbs().cwiseProduct(vel_cp)));
    Evec3 torque_drag = cenpres.cross(force_drag) - density_env*Cq*avel;

    // Fictitious
    Equat eci_body = ori_inv * geoid.enu_eci.inverse();
    Evec3 w_body = eci_body * geoid.w;
    Evec3 centripetal = eci_body * (geoid.w.cross(geoid.w.cross(geoid.eci_from_enu(pos))));
    Evec3 coriolis = (2*w_body + avel).cross(vel);
    Evec3 fict_linear = centripetal + coriolis - ori_inv*gravity_env;
    Evec3 fict_angular = w_body.cross(avel);

    // Proper accelerations
    veldot_inr = (force_buoy + force_act + force_drag) / mass;
    aveldot_inr = inertia_inv*(torque_buoy + torque_act + torque_drag - avel_inr.cross(inertia*avel_inr));

    // Collision mechanics (will eventually query Octree)
    Escal ground_penetration = craft->world->env.octree.ground_alt - lla.alt;
    if(ground_penetration > 0) {
        Escal isp = bounciness*ground_penetration/dt - (ori*vel).dot(local_up);
        if(isp > 0) {
            veldot_inr += ori_inv*((isp/dt)*local_up);
        }
    }

    // Accelerations
    veldot = veldot_inr - fict_linear;
    aveldot = aveldot_inr - fict_angular;

    // Update states
    pos += ori*(dt*vel + 0.5*dt*dt*veldot); // verlet
    pos_ecef = geoid.ecef_from_enu(pos);
    local_up = ori*fict_linear.normalized();
    lla = geoid.lla_from_ecef(pos_ecef);
    ori = ori * math::qexpmap(dt*avel + 0.5*dt*dt*aveldot); // local linearization
    ori_inv = ori.inverse();
    vel += dt*veldot; // RK1
    vel_rel = vel - ori_inv*wind_env - wind_act;
    avel += dt*aveldot; // RK1
    avel_inr = w_body + avel;
}

//////////////////////////////////////////////////

} // namespace physics
} // namespace stokesim
