/** @file
Declaration of the Sensor class.
*/
#pragma once
#include "stokesim/common.h"
#include "stokesim/math/so3.h"
#include "stokesim/math/gaussian.h"

namespace stokesim {
namespace physics {
class Craft;

//////////////////////////////////////////////////

/// Base class for a sensor: something that provides noisy,
/// finitely-sampled measurements of the World's contents.
class Sensor {
public:
    /// @name Core
    //@{
    Sensor(YAML::Node const& config, Craft const* container);
    //@}

    /// @name Provided
    //@{
    Craft const* craft; ///< Pointer to the owner of this sensor
    Str const    type; ///< Name of the derived class implementing this sensor
    Escal const  freq; ///< Measurement update frequency
    //@}

    /// @name Derived
    //@{
    Dict<Escal>      measurements; ///< Dictionary of the current measurement values
    std::vector<Str> measurement_names; ///< List of the valid keys for the measurements map
    Dict<Escal>      states; ///< Dictionary of the optionally exposed internal state values
    std::vector<Str> state_names; ///< List of the valid keys for the states map
    Escal const      period; ///< Reciprocal of freq, i.e. the time between measurements
    Escal            timestamp; ///< Sim time of latest measurement update
    uint32_t         sequence_number; ///< Integer sequence that increments whenever a new measurement is taken
    bool             killed; ///< Whether or not this sensor has been killed by a trigger, other plugin, etc
    //@}

    /// @name Types
    //@{
    struct Cmd {
        bool interrupt = false; ///< Trigger an immediate update (like for an interrupt-driven sensor)
        bool update_sequence = true; ///< Whether or not to update the sequence_number upon new measurement
        Dict<Escal> values; ///< Dictionary of remote / external values that affect the sensor
        Opt<bool> kill; ///< Set (true) or unset (false) the kill state of this sensor
        void merge(Cmd const& other);
    };
    //@}

    /// @name Access
    //@{
    inline bool  has_measurement(Str const& name) const {return measurements.count(name);}
    inline Escal get_measurement(Str const& name) const {return measurements.at(name);}
    inline bool  has_states() const {return state_names.size();}
    inline bool  has_state(Str const& name) const {return states.count(name);}
    inline Escal get_state(Str const& name) const {return states.at(name);}
    //@}

protected:
    /// @name Core
    //@{
    friend class Craft;
    virtual void update(Cmd const& cmd);
    virtual void update_states(Cmd const& cmd, Escal dt) =0; ///< Evolves any time-varying internal states (like noise / biases)
    virtual void update_measurements(Cmd const& cmd) =0; ///< Computes latest values for measurements dictionary
    //@}

    NO_COPY(Sensor)
};

//////////////////////////////////////////////////

} // namespace physics
} // namespace stokesim
