/** @file
Common dependencies, aliases, constants, and other
boiler-plate code for all C++ in this simulator project.
*/
#pragma once
#include <iostream>
#include <memory>
#include <vector>
#include <map>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

namespace stokesim {

//////////////////////////////////////////////////

/// @name Standard Aliases
//@{
using Str = std::string;
template <class T> using Dict = std::map<Str, T>;
template <class T> using Pdict = Dict<std::shared_ptr<T>>;
//@}

/// @name Eigen Aliases
//@{
#ifndef STOKESIM_SCALAR_TYPE
    #define STOKESIM_SCALAR_TYPE double
#endif
using Escal = STOKESIM_SCALAR_TYPE;
using Evec3 = Eigen::Matrix<Escal, 3, 1>;
using Emat3 = Eigen::Matrix<Escal, 3, 3>;
using Equat = Eigen::Quaternion<Escal>;
//@}

//////////////////////////////////////////////////

/// @name Constants
//@{
static constexpr Escal INF     = std::numeric_limits<Escal>::infinity();
static constexpr Escal EPS     = std::numeric_limits<Escal>::epsilon();
static constexpr Escal PI      = 3.1415926535897932;
static constexpr Escal RAD2DEG = 180/PI;
static constexpr Escal DEG2RAD = PI/180;
//@}

//////////////////////////////////////////////////

/// @name Primitive Containers
//@{

/// Latitude, longitude, and altitude POD
struct LLA {
    Escal lat;
    Escal lon;
    Escal alt;
    LLA() =default;
    LLA(Escal lat, Escal lon, Escal alt) :
        lat(lat),
        lon(lon),
        alt(alt) {
    }
};

/// Euler-angles POD
struct Euler {
    Escal roll;
    Escal pitch;
    Escal yaw;
    Euler() =default;
    Euler(Escal roll, Escal pitch, Escal yaw) :
        roll(roll),
        pitch(pitch),
        yaw(yaw) {
    }
};

/// Optionally provided value
template <class T>
class Opt {
private:
    bool _given;
    T _value;
public:
    Opt() : _given(false) {}
    Opt(T const& value) : _given(true), _value(value) {}
    T const& operator=(T const& value) {_given = true; _value = value; return _value;}
    operator T&() {assert(_given); return _value;}
    operator T() const {assert(_given); return _value;}
    bool given() const {return _given;}
};

//@}

//////////////////////////////////////////////////

/// Macro for disallowing copying an object of the given class type
#define NO_COPY(CLASS) \
    CLASS(CLASS const&) =delete;\
    CLASS & operator=(CLASS const&) =delete;\

//////////////////////////////////////////////////

/// @name YAML Helpers
//@{

/// Safely retrieves an entry from a YAML::Node with string keys
inline YAML::Node yget(Str const& key, YAML::Node const& node) {
    YAML::Node const& subnode = node[key];
    if(subnode) {return subnode;}
    throw std::runtime_error("YAML node is missing data with key '"+key+"'.");
}

/// Retrieves an entry from a YAML::Node with string keys if it exists, else uses default
template <class T>
inline YAML::Node yget(Str const& key, YAML::Node const& node, T default_value) {
    YAML::Node const& subnode = node[key];
    if(subnode.IsDefined() && !subnode.IsNull()) {return subnode;}
    return YAML::Node(default_value);
}

/// Converts YAML::Node to Eigen::Vector
inline Eigen::Matrix<Escal, Eigen::Dynamic, 1> evec_from_yaml(YAML::Node const& node, uint rows) {
    if(!node.IsSequence() || node.size()!=rows) {
        throw std::runtime_error("YAML::Node could not be converted to Eigen::Vector of length "+std::to_string(rows)+".");
    }
    Eigen::Matrix<Escal, Eigen::Dynamic, 1> evec(node.size());
    for(int i=0; i<evec.rows(); ++i) {evec(i) = node[i].as<Escal>();}
    return evec;
}

/// Converts YAML::Node to Eigen::Matrix
inline Eigen::Matrix<Escal, Eigen::Dynamic, Eigen::Dynamic> emat_from_yaml(YAML::Node const& node, uint rows, uint cols) {
    if(!node.IsSequence() || node.size()!=rows || !node[0].IsSequence() || node[0].size()!=cols) {
        throw std::runtime_error("YAML::Node could not be converted to Eigen::Matrix of shape ("+std::to_string(rows)+","+std::to_string(cols)+").");
    }
    Eigen::Matrix<Escal, Eigen::Dynamic, Eigen::Dynamic> emat(node.size(), node[0].size());
    for(int col=0; col<emat.cols(); ++col) {
        for(int row=0; row<emat.rows(); ++row) {
            emat(row, col) = node[row][col].as<Escal>();
        }
    }
    return emat;
}

/// Converts YAML::Node to Equat
inline Equat equat_from_yaml(YAML::Node const& node) {
    return Equat(yget("w", node).as<Escal>(),
                 yget("x", node).as<Escal>(),
                 yget("y", node).as<Escal>(),
                 yget("z", node).as<Escal>()).normalized();
}

/// Converts YAML::Node to LLA
inline LLA lla_from_yaml(YAML::Node const& node) {
    return LLA(yget("lat", node).as<Escal>(),
               yget("lon", node).as<Escal>(),
               yget("alt", node).as<Escal>());
}

/// Converts YAML::Node to Euler
inline Euler euler_from_yaml(YAML::Node const& node) {
    return Euler(yget("roll", node).as<Escal>(),
                 yget("pitch", node).as<Escal>(),
                 yget("yaw", node).as<Escal>());
}

//@}

//////////////////////////////////////////////////

/// @name Print Helpers
//@{

/// Quicker way to use std::cout for simple prints
template <class T>
inline void print(T const& obj) {
    std::cout << obj << std::endl;
}

/// Specifically prints 3-vectors horizontally
inline void print(Evec3 const& v) {
    std::cout << v.transpose() << std::endl;
}

/// Specifically prints quaternions in 1ijk format
inline void print(Equat const& q) {
    std::cout << Eigen::Vector4d(q.w(), q.x(), q.y(), q.z()).transpose() << std::endl;
}

/// Specifically prints LLAs
inline void print(LLA const& lla) {
    std::cout << "lat: " << lla.lat << " | lon: " << lla.lon << " | alt: " << lla.alt << std::endl;
}

/// Specifically prints Euler angles
inline void print(Euler const& euler) {
    std::cout << "roll: " << euler.roll << " | pitch: " << euler.pitch << " | yaw: " << euler.yaw << std::endl;
}

//@}

//////////////////////////////////////////////////

} // namespace stokesim
