/** @file
Basic math operations that are not in the C++11 standard libraries.
*/
#pragma once
#include "stokesim/common.h"

namespace stokesim {
namespace math {

//////////////////////////////////////////////////

/// Clips a number n between lo and hi
template <typename T>
inline T clip(T n, T lo, T hi) {
    if(n > hi) {return hi;}
    if(n < lo) {return lo;}
    return n;
}

//////////////////////////////////////////////////

/// Returns the sign of a number n
template <typename T>
inline T signum(T n) {
    static constexpr T zero(0);
    return (zero<n) - (n<zero);
}

//////////////////////////////////////////////////

/// Computes the cardinal sine function of a real number x
inline Escal sinc(Escal x) {
    if(fabs(x) < EPS) {return 1;}
    return sin(x)/x;
}

//////////////////////////////////////////////////

} // namespace math
} // namespace stokesim
