/** @file
Declaration of the Craft class.
*/
#pragma once
#include "stokesim/common.h"
#include "stokesim/physics/body.h"
#include "stokesim/physics/actuator_types/types.h"
#include "stokesim/physics/sensor_types/types.h"

namespace stokesim {
namespace physics {
class World;

//////////////////////////////////////////////////

/// An air/marine-craft composed of a body, actuators,
/// sensors, and an energy supply (defaults infinite).
class Craft {
private:
    /// @name Provided
    //@{
    Pdict<Actuator> actuators; ///< Map from actuator names to shared pointers at each actuator
    Pdict<Sensor>   sensors; ///< Map from sensor names to shared pointers at each sensor
public:
    World const*    world; ///< Owner of this craft
    Body            body;
    Escal const     energy0; ///< Initial total energy of the supply (>0) (DEFAULT: INF)
    Escal const     voltage0; ///< Nominal voltage of the supply (>0) (DEFAULT: 1 V)
    Escal const     power0; ///< Constant power-draw by the craft electronics but not actuators (>=0) (DEFAULT: 0 W)
    //@}

    /// @name Derived
    //@{
    std::vector<Str> actuator_names; ///< List of all the keys for the actuators map
    std::vector<Str> sensor_names; ///< List of all the keys for the sensors map
    Escal energy; ///< Energy remaining in the supply
    Escal power; ///< Total instantaneous power-draw by craft electronics and actuators
    bool killed; ///< Whether or not this craft has been killed by a trigger, other plugin, etc
    Equat const flu_frd; ///< Orientation of forward_left_up relative to forward_right_down
    //@}

    /// @name Utilities
    //@{
    Evec3 flu_from_frd(Evec3 const& p) const;
    Evec3 frd_from_flu(Evec3 const& p) const;
    //@}

    /// @name Types
    //@{
    struct Cmd {
        Body::Cmd body_cmd; ///< Command to pass on to this craft's body
        Dict<Actuator::Cmd> actuator_cmds; ///< Commands to pass on to this craft's actuators
        Dict<Sensor::Cmd> sensor_cmds; ///< Commands to pass on to this craft's sensors
        Opt<bool> kill; ///< Set (true) or unset (false) the kill state of this craft
        void merge(Cmd const& other);
    };
    //@}

    /// @name Access
    //@{
    inline Actuator const& get_actuator(Str const& name) const {return *actuators.at(name);}
    inline Sensor   const& get_sensor(Str const& name)   const {return *sensors.at(name);}
    inline bool            has_actuator(Str const& name) const {return actuators.count(name);}
    inline bool            has_sensor(Str const& name)   const {return sensors.count(name);}
    //@}

private:
    /// @name Core
    //@{
    friend class World;
    Craft(YAML::Node const& config, World const* container);
    void update(Cmd const& cmd);
    //@}

    NO_COPY(Craft)
};

//////////////////////////////////////////////////

} // namespace physics
} // namespace stokesim
