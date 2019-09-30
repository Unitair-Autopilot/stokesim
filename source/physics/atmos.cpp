/** @file
Implementation of atmos.h
*/
#include "stokesim/physics/atmos.h"
#include "stokesim/physics/world.h"

namespace stokesim {
namespace physics {

//////////////////////////////////////////////////

Evec3 Atmos::wind_at_lla(LLA const& lla) const {
    // Take wind stochastic parameters to be in instantly local ENU, convert to ECEF
    return env->geoid.local_at_lla(lla) * (wind_mean + turb + gust);
}

//////////////////////////////////////////////////

Escal Atmos::temperature_at_alt(Escal alt) const {
    // If above ocean, ISA linear formula
    if(alt > 0) {return T0 - L*alt;}
    // Otherwise, assume sea-level temperature throughout (should be ocean temperature model)
    return T0;
}

//////////////////////////////////////////////////

Escal Atmos::pressure_at_alt(Escal alt) const {
    // If above ocean, ISA polynomial formula
    if(alt > 0) {return P0 * pow(1 - L*alt/T0, x);}
    // Otherwise, basic hydrostatics
    return P0 - rho*g*alt;
}

//////////////////////////////////////////////////

Escal Atmos::density_at_alt(Escal alt) const {
    // If above ocean, ideal gas law
    if(alt > 0) {return (M * pressure_at_alt(alt)) / (R * temperature_at_alt(alt));}
    // Otherwise, smoothed transition to ocean density (for numerical stability at boundary)
    return (rho-1.225)*(1-exp(10*alt)) + 1.225;
}

//////////////////////////////////////////////////

void Atmos::Cmd::merge(Cmd const& other) {
    if(other.gust_mag >= 0) {
        gust_mag = other.gust_mag;
        gust_dir = other.gust_dir;
    }
}

//////////////////////////////////////////////////

Atmos::Atmos(YAML::Node const& config, Env const* container) :
    env(container),
    P0(yget("P0", config, 101.325e3).as<Escal>()),
    T0(yget("T0", config, 288.15).as<Escal>()),
    L(yget("L", config, 0.0065).as<Escal>()),
    R(yget("R", config, 8.31447).as<Escal>()),
    M(yget("M", config, 0.0289644).as<Escal>()),
    g(yget("g", config, 9.80665).as<Escal>()),
    rho(yget("rho", config, 1023.6).as<Escal>()),
    wind_mean(evec_from_yaml(yget("wind_mean", config), 3)),
    turb_taus(evec_from_yaml(yget("turb_taus", config), 3)),
    turb_covar(emat_from_yaml(yget("turb_covar", config), 3, 3)),
    x(g*M/(R*L)),
    turb_springs(turb_taus.cwiseInverse()),
    turb_pdf(Evec3(0, 0, 0), turb_covar),
    turb(0, 0, 0),
    gust(0, 0, 0) {
}

//////////////////////////////////////////////////

void Atmos::update(Cmd const& cmd) {
    // Alias integration timestep
    Escal dt = env->world->clock.dt;
    // Update turbulence stochastic process (Ornstein noise)
    turb += sqrt(dt)*turb_pdf.samples(1) - dt*turb_springs.cwiseProduct(turb);
    // Set gust to commanded gust
    if(cmd.gust_mag >= 0) {
        gust = cmd.gust_mag * cmd.gust_dir.normalized();
    }
}

//////////////////////////////////////////////////

} // namespace physics
} // namespace stokesim
