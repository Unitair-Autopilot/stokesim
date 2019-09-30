/** @file
Declaration of the Geoid class.
*/
#pragma once
#include "stokesim/common.h"
#include "stokesim/math/so3.h"

namespace stokesim {
namespace physics {
class Env;

//////////////////////////////////////////////////

/// The ellipsoidal body of a spinning planet. Manages inertial
/// and geodetic coordinates, and gravity. All defaults are WGS84.
class Geoid {
public:
    /// @name Provided
    //@{
    Env const*  env; ///< Pointer to the owner of this geoid
    Escal const a; ///< Semi-major axis (DEFAULT: 6378137 m)
    Escal const f; ///< Flattening factor (DEFAULT: 1/298.257223563)
    Escal const u; ///< Gravitational constant (DEFAULT: 3986004.418e8 m^3/s^2)
    Escal const c20; ///< Geopotential coefficient (DEFAULT: -1082.63e-6)
    Evec3 const w; ///< Angular velocity of ECEF frame relative to ECI frame (DEFAULT: [0, 0, 7.292e-5] rad/s)
    LLA const   enu0_lla; ///< LLA of ENU origin
    //@}

    /// @name Derived
    //@{
    Escal const b; ///< Semi-minor axis
    Escal const e2; ///< Eccentricity squared
    Escal const ep2; ///< Second-eccentricity squared
    Equat const enu_ned; ///< Orientation of east_north_up relative to north_east_down
    Evec3 const enu0_ecef; ///< Position of the ENU origin in ECEF coordinates
    Equat const enu_ecef; ///< Orientation of east_north_up relative to earth-centered-earth-fixed coordinates
    Equat const ecef_enu; ///< Inverse of enu_ecef
    Equat ecef_eci; ///< Current orientation of ECEF relative to earth-centered-inertial
    Evec3 enu0_eci; ///< Current position of the ENU origin in ECI coordinates
    Equat enu_eci; ///< Current orientation of ENU relative to ECI
    //@}

    /// @name Utilities
    //@{
    Evec3 gravity_at_ecef(Evec3 const& p) const; ///< Computes the gravity field vector in ECEF at the given ECEF point
    Equat local_at_lla(LLA const& lla) const; ///< Computes the orientation of local-ENU relative to ECEF at a given LLA
    ////
    Evec3 ecef_from_lla(LLA lla, bool use_degrees=true) const;
    LLA   lla_from_ecef(Evec3 const& p, bool use_degrees=true) const;
    ////
    Evec3 ecef_from_enu(Evec3 const& p) const;
    Evec3 enu_from_ecef(Evec3 const& p) const;
    ////
    Evec3 eci_from_enu(Evec3 const& p) const;
    Evec3 enu_from_eci(Evec3 const& p) const;
    ////
    LLA   lla_from_enu(Evec3 const& p, bool use_degrees=true) const;
    Evec3 enu_from_lla(LLA const& lla, bool use_degrees=true) const;
    ////
    Evec3 ned_from_enu(Evec3 const& p) const;
    Evec3 enu_from_ned(Evec3 const& p) const;
    //@}

    /// @name Types
    //@{
    struct Cmd {inline void merge(Cmd const& other) {}};
    //@}

private:
    /// @name Core
    //@{
    friend class Env;
    Geoid(YAML::Node const& config, Env const* container);
    void update(Cmd const& cmd);
    //@}

    NO_COPY(Geoid)
};

//////////////////////////////////////////////////

} // namespace physics
} // namespace stokesim
