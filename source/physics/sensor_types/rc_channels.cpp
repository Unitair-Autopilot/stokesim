/** @file
Implementation of rc_channels.h
*/
#include "stokesim/physics/sensor_types/rc_channels.h"
#include "stokesim/physics/world.h"

namespace stokesim {
namespace physics {
namespace sensor_types {

//////////////////////////////////////////////////

RcChannels::RcChannels(YAML::Node const& config, Craft const* container) :
    Sensor(config, container),
    num_channels(yget("num_channels", config).as<int>()) {
    for(int i=1; i<=num_channels; ++i) {
        measurement_names.push_back("chan_" + std::to_string(i));
    }
    if(config["default_channel_value"]) {
        double default_channel_value = yget("default_channel_value", config).as<double>();
        for(auto const& name : measurement_names) {
            measurements[name] = default_channel_value;
        }
        measurement_names.push_back("rssi");
        measurement_names.push_back("chancount");
        measurements["rssi"] = 100.0;
        measurements["chancount"] = static_cast<double>(num_channels);
    } else {
        bool defaults_present = false;
        for(auto const& name : measurement_names) {
            if(config[name]) {
                defaults_present = true;
                measurements[name] = yget(name, config).as<double>();
            } else {
                measurements[name] = 0.0;
            }
        }
        measurement_names.push_back("rssi");
        measurement_names.push_back("chancount");
        if(defaults_present) {
            measurements["rssi"] = 100.0;
            measurements["chancount"] = static_cast<double>(num_channels);
        } else {
            measurements["rssi"] = 0.0;
            measurements["chancount"] = 0.0;
        }
    }
}

//////////////////////////////////////////////////

void RcChannels::update_states(Cmd const& cmd, Escal dt) {}

//////////////////////////////////////////////////

void RcChannels::update_measurements(Cmd const& cmd) {
    for(auto const& pair : cmd.values) {
        if(measurements.count(pair.first)) {
            measurements[pair.first] = pair.second;
        }
    }
}

//////////////////////////////////////////////////

}  // namespace sensor_types
}  // namespace physics
}  // namespace stokesim
