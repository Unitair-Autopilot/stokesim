/** @file
Implementation of geoid.h
*/
#include "stokesim/physics/geoid.h"
#include "stokesim/physics/world.h"

namespace stokesim {
namespace physics {

//////////////////////////////////////////////////

Evec3 Geoid::gravity_at_ecef(Evec3 const& p) const {
    // WGS84 gravity model using ECEF coordinates
    Escal dist = p.norm();
    Evec3 dir = p / dist;
    Escal z2 = pow(dir(2), 2);
    Escal ubar = u / pow(dist, 2);
    Escal rho2 = pow(a/dist, 2);
    return -ubar*dir + (1.5*c20*ubar*rho2)*dir.cwiseProduct(Evec3(1-5*z2, 1-5*z2, 3-5*z2));
}

//////////////////////////////////////////////////

Equat Geoid::local_at_lla(LLA const& lla) const {
    return math::quat_from_euler(Euler(90-lla.lat, 0, 90+lla.lon));
}

//////////////////////////////////////////////////

Evec3 Geoid::ecef_from_lla(LLA lla, bool use_degrees/*=true*/) const {
    if(use_degrees) {
        lla.lat *= DEG2RAD;
        lla.lon *= DEG2RAD;
    }
    Escal c_lat = cos(lla.lat);
    Escal s_lat = sin(lla.lat);
    Escal c_lon = cos(lla.lon);
    Escal s_lon = sin(lla.lon);
    Escal n = a/sqrt(1-e2*pow(sin(lla.lat), 2));
    return Evec3((n + lla.alt)*c_lat*c_lon,
                 (n + lla.alt)*c_lat*s_lon,
                 (n*b*b/(a*a) + lla.alt)*s_lat);
}

//////////////////////////////////////////////////

LLA Geoid::lla_from_ecef(Evec3 const& p, bool use_degrees/*=true*/) const {
    LLA lla;
    Escal r = sqrt(p(0)*p(0) + p(1)*p(1));
    Escal th = atan2(a*p(2), b*r);
    lla.lat = atan2(p(2) + ep2*b*pow(sin(th), 3), r - e2*a*pow(cos(th), 3));
    lla.lon = fmod(atan2(p(1), p(0)), 2*PI);
    if(fabs(p(0))<1 && fabs(p(1))<1) {lla.alt = fabs(p(2)) - b;}
    else {lla.alt = r/cos(lla.lat) - a/sqrt(1-e2*pow(sin(lla.lat), 2));}
    if(use_degrees) {
        lla.lat *= RAD2DEG;
        lla.lon *= RAD2DEG;
    }
    return lla;
}

//////////////////////////////////////////////////

Evec3 Geoid::ecef_from_enu(Evec3 const& p) const {
    return enu0_ecef + enu_ecef*p;
}

//////////////////////////////////////////////////

Evec3 Geoid::enu_from_ecef(Evec3 const& p) const {
    return enu_ecef.inverse()*(p - enu0_ecef);
}

//////////////////////////////////////////////////

Evec3 Geoid::eci_from_enu(Evec3 const& p) const {
    return enu0_eci + enu_eci*p;
}

//////////////////////////////////////////////////

Evec3 Geoid::enu_from_eci(Evec3 const& p) const {
    return enu_eci.inverse()*(p - enu0_eci);
}

//////////////////////////////////////////////////

LLA Geoid::lla_from_enu(Evec3 const& p, bool use_degrees/*=true*/) const {
    return lla_from_ecef(ecef_from_enu(p), use_degrees);
}

//////////////////////////////////////////////////

Evec3 Geoid::enu_from_lla(LLA const& lla, bool use_degrees/*=true*/) const {
    return enu_from_ecef(ecef_from_lla(lla, use_degrees));
}

//////////////////////////////////////////////////

Evec3 Geoid::ned_from_enu(Evec3 const& p) const {
    return enu_ned*p;
}

//////////////////////////////////////////////////

Evec3 Geoid::enu_from_ned(Evec3 const& p) const {
    return enu_ned.inverse()*p;
}

//////////////////////////////////////////////////

Geoid::Geoid(YAML::Node const& config, Env const* container) :
    env(container),
    a(yget("a", config, 6378137).as<Escal>()),
    f(yget("f", config, 1.0/298.257223563).as<Escal>()),
    u(yget("u", config, 3986004.418e8).as<Escal>()),
    c20(yget("c20", config, -1082.63e-6).as<Escal>()),
    w(evec_from_yaml(yget("w", config, std::vector<Escal>({0, 0, 7.292e-5})), 3)),
    enu0_lla(lla_from_yaml(yget("enu0_lla", config))),
    b(a*(1-f)),
    e2(2*f-f*f),
    ep2(e2/(1-e2)),
    enu_ned(0, sqrt(2)/2, sqrt(2)/2, 0),
    enu0_ecef(ecef_from_lla(enu0_lla)),
    enu_ecef(local_at_lla(enu0_lla)),
    ecef_enu(enu_ecef.inverse()),
    ecef_eci(1, 0, 0, 0),
    enu0_eci(ecef_eci*enu0_ecef),
    enu_eci(ecef_eci*enu_ecef) {
}

//////////////////////////////////////////////////

void Geoid::update(Cmd const& cmd) {
    // ECEF is a constant-w rotation for t time relative to ECI
    ecef_eci = math::qexpmap(env->world->clock.t * w);
    // Compute ENU features relative to ECI by composing ENU-to-ECEF with ECEF-to-ECI
    enu0_eci = ecef_eci * enu0_ecef;
    enu_eci = ecef_eci * enu_ecef;
}

//////////////////////////////////////////////////

} // namespace physics
} // namespace stokesim
