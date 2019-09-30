/** @file
Declaration of the Body class.
*/
#pragma once
#include "stokesim/common.h"
#include "stokesim/math/so3.h"

namespace stokesim {
namespace physics {
class Craft;

//////////////////////////////////////////////////

/// A rigid-body.
class Body {
public:
    /// @name Provided
    //@{
    Craft const* craft; ///< Pointer to the owner of this body
    Evec3 const  pos0; ///< Initial position of the center-of-mass in ENU
    Equat const  ori0; ///< Initial orientation of COM-coordinates relative to ENU
    Evec3 const  vel0; ///< Initial velocity of the COM relative to geoid expressed in COM-coordinates
    Evec3 const  avel0; ///< Initial angular velocity of body relative to geoid expressed in COM-coordinates
    Escal const  mass; ///< Total mass (including other parts of owner craft)
    Emat3 const  inertia; ///< Inertia tensor (including other parts of owner craft) expressed in COM-coordinates
    Evec3 const  centroid; ///< Position of the geometric centroid in COM-coordinates
    Evec3 const  bounding_box; ///< Edge dimensions of COM-coordinate-aligned bounding box
    Evec3 const  areas; ///< Largest cross-sectional area along each COM-coordinate axis
    Escal const  volume; ///< Total volume (including other parts of owner craft)
    Evec3 const  cenpres; ///< Position of the center of pressure of the body (no actuators) in COM-coordinates
    Emat3 const  Cp1; ///< Translational linear-drag tensor in COM-coordinates
    Emat3 const  Cp2; ///<  Translational quadratic-drag tensor in COM-coordinates
    Emat3 const  Cq; ///< Rotational linear-drag tensor in COM-coordinates
    Escal const  bounciness; ///< Coefficient of bounce for impacts, 0 to 1 (DEFAULT: 0.001)
    //@}

    /// @name Derived
    //@{
    LLA   lla; ///< Current LLA of COM
    Evec3 local_up; ///< Level-defined "up" at the current COM position, expressed in ENU coordinates
    Evec3 pos_ecef; ///< Current position of the center-of-mass in ECEF
    Evec3 pos; ///< Current position of the center-of-mass in ENU
    Equat ori; ///< Current orientation of COM-coordinates relative to ENU
    Equat ori_inv; ///< Inverse of ori
    Evec3 vel; ///< Current velocity of the COM relative to geoid expressed in COM-coordinates
    Evec3 vel_rel; ///< Current velocity relative to the local wind vector, i.e. vel - wind
    Evec3 avel; ///< Current angular velocity of body relative to geoid expressed in COM-coordinates
    Evec3 avel_inr; ///< Current inertial angular velocity of body relative to geoid expressed in COM-coordinates
    Evec3 veldot; ///< Translational acceleration of COM relative to geoid expressed in COM-coordinates
    Evec3 veldot_inr; ///< Inertial (proper) translational acceleration of COM relative to geoid expressed in COM-coordinates
    Evec3 aveldot; ///< Angular acceleration of body relative to geoid expressed in COM-coordinates
    Evec3 aveldot_inr; ///< Inertial (proper) angular acceleration of body relative to geoid expressed in COM-coordinates
    Emat3 const inertia_inv; ///< Matrix inverse of inertia tensor in COM-coordinates
    //@}

    /// @name Utilities
    //@{
    Euler compute_euler(bool use_degrees=true) const; ///< Computes the Z-Y'-X'' Euler angles that compose ori
    Escal compute_ang_attack(bool use_degrees=true) const; ///< Computes the current angle of attack
    Escal compute_ang_sideslip(bool use_degrees=true) const; ///< Computes the current angle of sideslip
    Escal alt_of_point(Evec3 const& p) const; ///< Computes the altitude of a point fixed to the body in COM-coordinates
    //@}

    /// @name Types
    //@{
    struct Cmd {inline void merge(Cmd const& other) {}};
    //@}

private:
    /// @name Core
    //@{
    friend class Craft;
    Body(YAML::Node const& config, Craft const* container);
    void update(Cmd const& cmd);
    //@}

    NO_COPY(Body)
};

//////////////////////////////////////////////////

} // namespace physics
} // namespace stokesim
