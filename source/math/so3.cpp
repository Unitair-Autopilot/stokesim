/** @file
Implementation of so3.h
*/
#include "stokesim/math/so3.h"

namespace stokesim {
namespace math {

//////////////////////////////////////////////////

Equat qexpmap(Evec3 const& r) {
    Escal ang = r.norm();
    Equat q;
    q.w() = cos(ang/2);
    q.vec() = sinc(ang/2)/2 * r;
    return q;
}

//////////////////////////////////////////////////

Evec3 qlogmap(Equat q) {
    q.normalize();
    if(q.w() < 0) {q.coeffs() = -q.coeffs();}
    return 2/sinc(acos(std::min(1.0, q.w()))) * q.vec();
}

//////////////////////////////////////////////////

Euler euler_from_quat(Equat const& q, bool use_degrees/*=true*/) {
    Euler euler;
    euler.roll = atan2(2.0*(q.w()*q.x() + q.y()*q.z()),
                       1.0 - 2.0*(q.x()*q.x() + q.y()*q.y()));
    Escal sinp = 2.0*(q.w()*q.y() - q.z()*q.x());
    if(fabs(sinp) >= 1-EPS) {
        euler.pitch = copysign(PI/2, sinp);
    } else {
        euler.pitch = asin(sinp);
    }
    euler.yaw = atan2(2.0*(q.w()*q.z() + q.x()*q.y()),
                      1.0 - 2.0*(q.y()*q.y() + q.z()*q.z()));
    if(use_degrees) {
        euler.roll *= RAD2DEG;
        euler.pitch *= RAD2DEG;
        euler.yaw *= RAD2DEG;
    }
    return euler;
}

//////////////////////////////////////////////////

Equat quat_from_euler(Euler const& euler, bool use_degrees/*=true*/) {
    Euler half_euler;
    if(use_degrees) {
        half_euler.roll = 0.5*DEG2RAD*euler.roll;
        half_euler.pitch = 0.5*DEG2RAD*euler.pitch;
        half_euler.yaw = 0.5*DEG2RAD*euler.yaw;
    } else {
        half_euler.roll = 0.5*euler.roll;
        half_euler.pitch = 0.5*euler.pitch;
        half_euler.yaw = 0.5*euler.yaw;
    }
    Escal cy = cos(half_euler.yaw);
    Escal sy = sin(half_euler.yaw);
    Escal cr = cos(half_euler.roll);
    Escal sr = sin(half_euler.roll);
    Escal cp = cos(half_euler.pitch);
    Escal sp = sin(half_euler.pitch);
    return Equat(cy*cr*cp + sy*sr*sp,  // 1
                 cy*sr*cp - sy*cr*sp,  // i
                 cy*cr*sp + sy*sr*cp,  // j
                 sy*cr*cp - cy*sr*sp); // k
}

//////////////////////////////////////////////////

Escal amod(Escal ang, bool use_degrees/*=false*/) {
    auto sign = signum(ang);
    if(use_degrees) {return fmod(ang + sign*180, 360) - sign*180;}
    return fmod(ang + sign*PI, 2*PI) - sign*PI;
}

//////////////////////////////////////////////////

} // namespace math
} // namespace stokesim
