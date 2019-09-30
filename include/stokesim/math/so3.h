/** @file
Functions for handling the unit-quaternion double-cover
of the Lie group SO3. Much of the code that would usually
be implemented in a file like this is already nicely provided
by Eigen::Quaternion.
*/
#pragma once
#include "stokesim/common.h"
#include "stokesim/math/basic.h"

namespace stokesim {
namespace math {

//////////////////////////////////////////////////

/// Converts an angle*axis vector (in radians) to a quaternion
Equat qexpmap(Evec3 const& r);

/// Converts a quaternion to an angle*axis vector (in radians)
Evec3 qlogmap(Equat q);

/// Converts a quaternion to Euler angles in k-j'-i'' (aerospace) convention
Euler euler_from_quat(Equat const& q, bool use_degrees=true);

/// Converts Euler angles in k-j'-i'' (aerospace) convention to a quaternion
Equat quat_from_euler(Euler const& euler, bool use_degrees=true);

/// Maps an angle (SO2 minimal representation) onto [-pi, pi] or [-180, 180]
Escal amod(Escal ang, bool use_degrees=false);

//////////////////////////////////////////////////

} // namespace math
} // namespace stokesim
