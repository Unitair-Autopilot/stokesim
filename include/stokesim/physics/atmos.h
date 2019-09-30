/** @file
Declaration of the Atmos class.
*/
#pragma once
#include "stokesim/common.h"
#include "stokesim/math/gaussian.h"

namespace stokesim {
namespace physics {
class Env;

//////////////////////////////////////////////////

/// Atmosphere. Manages pressure, temperature, density, and velocity
/// of the surrounding fluid. Additional gusts can be commanded.
/// All defaults are ISA (ISO 2533:1975).
class Atmos {
public:
    /// @name Provided
    //@{
    Env const*  env; ///< Pointer to the owner of this atmos
    Escal const P0; ///< Sea-level standard pressure (DEFAULT: 101.325e3 Pa)
    Escal const T0; ///< Sea-level standard temperature (DEFAULT: 288.15 K)
    Escal const L; ///< Temperature lapse rate (DEFAULT: 0.0065 K/m)
    Escal const R; ///< Ideal gas constant (DEFAULT: 8.31447 J/(mol*K))
    Escal const M; ///< Molar mass of dry air (DEFAULT: 0.0289644 kg/mol)
    Escal const g; ///< Standard gravity (DEFAULT: 9.80665 m/s^2)
    Escal const rho; ///< Density of sea (DEFAULT: 1023.6 kg/m^3)
    Evec3 const wind_mean; ///< Uniform wind mean velocity in local ENU coordinates
    Evec3 const turb_taus; ///< Time-constants measuring the mean-return tendency of the wind field
    Emat3 const turb_covar; ///< Wind turbulence random walk covariance matrix
    //@}

    /// @name Derived
    //@{
    Escal const    x; ///< Atmospheric exponent
    Evec3 const    turb_springs; ///< Reciprocals of turb_taus
    math::Gaussian turb_pdf; ///< Probability density function associated with turb_covar
    Evec3          turb; ///< Current turbulence vector
    Evec3          gust; ///< Current gust vector
    //@}

    /// @name Utilities
    //@{
    Evec3 wind_at_lla(LLA const& lla) const; ///< Computes the wind velocity vector in ECEF at the given LLA
    Escal temperature_at_alt(Escal alt) const; ///< Computes the temperature at the given altitude
    Escal pressure_at_alt(Escal alt) const; ///< Computes the pressure at the given altitude
    Escal density_at_alt(Escal alt) const; ///< Computes the fluid density at the given altitude
    //@}

    /// @name Types
    //@{
    struct Cmd {
        Escal gust_mag = -1; ///< Magnitude of the vector added to the wind field everywhere (negative means passive, i.e. no overwrite)
        Evec3 gust_dir = Evec3(1, 0, 0); ///< Direction of the vector added to the wind field everywhere
        void merge(Cmd const& other);
    };
    //@}

private:
    /// @name Core
    //@{
    friend class Env;
    Atmos(YAML::Node const& config, Env const* container);
    void update(Cmd const& cmd);
    //@}

    NO_COPY(Atmos)
};

//////////////////////////////////////////////////

} // namespace physics
} // namespace stokesim
