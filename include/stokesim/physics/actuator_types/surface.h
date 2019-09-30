/** @file
Declaration of the Surface class.
*/
#pragma once
#include "stokesim/physics/actuator.h"

namespace stokesim {
namespace physics {
class Craft;
namespace actuator_types {

//////////////////////////////////////////////////

/// Flight surface, which can be rotated at a finite slew-rate about some
/// body-fixed axis (up to some limits) to change the positioning of its
/// center of pressure and the orientation of its drag-inducing face.
/// Surfaces are assumed to have negligible inertia.
class Surface : public Actuator {
public:
    /// @name Provided
    //@{
    std::vector<Escal> const dims; ///< Lengths along the surface 0 and 1 directions
    Escal const coeff; ///< Coefficient for drag inducing face (>0)
    Evec3 const origin; ///< Mounting point and origin of surface-coordinates in COM-coordinates
    Equat const orient0; ///< Trim orientation of surface-coordinates relative to COM-coordinates
    Evec3 const cenpres; ///< Vector in surface-coordinates of the center of pressure
    Evec3 const axis; ///< Direction vector in surface-coordinates about which the surface can be rotated
    Escal const ang_min; ///< Minimum deflection angle, deg
    Escal const ang_max; ///< Maximum deflection angle, deg
    Escal const angdot; ///< Angular velocity magnitude for rotating the surface, deg per unit time (>0)
    Escal const flutter_amp; ///< Amplitude of flutter deflections, deg per unit speed (DEFAULT: 0)
    Escal const flutter_freq; ///< Frequency of flutter deflections, Hz (DEFAULT: 0)
    //@}

    /// @name Derived
    //@{
    Escal const area; ///< Product of dims
    Evec3 const normal; ///< Always [0, 0, 1] in surface-coordinates
    Escal ang_cmd; ///< Current (latched) commanded deflection angle
    Escal ang; ///< Current attained deflection angle
    Equat orient; ///< Current orientation of surface-coordinates relative to COM-coordinates
    //@}

private:
    /// @name Core
    //@{
    friend class stokesim::physics::Craft;
    Surface(YAML::Node const& config, Craft const* container);
    void update_impact(Cmd const& cmd) override;
    void set_effort(Escal effort) override; ///< Effort is on [-1, 1]
    //@}

    NO_COPY(Surface)
};

//////////////////////////////////////////////////

} // namespace actuator_types
} // namespace physics
} // namespace stokesim
