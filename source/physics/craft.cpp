/** @file
Implementation of craft.h
*/
#include "stokesim/physics/craft.h"
#include "stokesim/physics/world.h"

namespace stokesim {
namespace physics {

//////////////////////////////////////////////////

Evec3 Craft::flu_from_frd(Evec3 const& p) const {
    return flu_frd*p;
}

//////////////////////////////////////////////////

Evec3 Craft::frd_from_flu(Evec3 const& p) const {
    return flu_frd.inverse()*p;
}

//////////////////////////////////////////////////

void Craft::Cmd::merge(Cmd const& other) {
    body_cmd.merge(other.body_cmd);
    for(auto const& pair : other.actuator_cmds) {actuator_cmds[pair.first].merge(pair.second);}
    for(auto const& pair : other.sensor_cmds) {sensor_cmds[pair.first].merge(pair.second);}
    if(other.kill.given()) {kill = other.kill;}
}

//////////////////////////////////////////////////

Craft::Craft(YAML::Node const& config, World const* container) :
    world(container),
    body(yget("body", config), this),
    energy0(yget("energy0", config, 1e99).as<Escal>()), // (std-limits doesn't play well with older yaml-cpp versions)
    voltage0(yget("voltage0", config, 1).as<Escal>()),
    power0(yget("power0", config, 0).as<Escal>()),
    energy(energy0),
    power(power0),
    killed(false),
    flu_frd(0, 1, 0, 0) {
    // For each actuator specification
    for(auto const& pair : yget("actuators", config)) {
        // Extract the actuator's name and config yaml
        Str const& name = pair.first.as<Str>();
        YAML::Node const& actuator_config = pair.second;
        // Add the name to the names list
        actuator_names.push_back(name);
        // Create a new actuator-derived object depending on the type specification
        Str type = yget("type", actuator_config).as<Str>();
        if(type == "thruster") {
            actuators.insert({name, std::shared_ptr<Actuator>(new actuator_types::Thruster(actuator_config, this))});
        } else if(type == "surface") {
            actuators.insert({name, std::shared_ptr<Actuator>(new actuator_types::Surface(actuator_config, this))});
        } else if(type == "launcher") {
            actuators.insert({name, std::shared_ptr<Actuator>(new actuator_types::Launcher(actuator_config, this))});
        } else {
            throw std::runtime_error("Actuator type '"+type+"' has not been implemented.");
        }
    }
    // For each sensor specification
    for(auto const& pair : yget("sensors", config)) {
        // Extract the sensor's name and config yaml
        Str const& name = pair.first.as<Str>();
        YAML::Node const& sensor_config = pair.second;
        // Add the name to the names list
        sensor_names.push_back(name);
        // Create a new sensor-derived object depending on the type specification
        Str type = yget("type", sensor_config).as<Str>();
        if(type == "accelerometer") {
            sensors.insert({name, std::shared_ptr<Sensor>(new sensor_types::Accelerometer(sensor_config, this))});
        } else if(type == "barometer") {
            sensors.insert({name, std::shared_ptr<Sensor>(new sensor_types::Barometer(sensor_config, this))});
        } else if(type == "gps") {
            sensors.insert({name, std::shared_ptr<Sensor>(new sensor_types::GPS(sensor_config, this))});
        } else if(type == "gyroscope") {
            sensors.insert({name, std::shared_ptr<Sensor>(new sensor_types::Gyroscope(sensor_config, this))});
        } else if(type == "imu") {
            sensors.insert({name, std::shared_ptr<Sensor>(new sensor_types::IMU(sensor_config, this))});
        } else if(type == "magnetometer") {
            sensors.insert({name, std::shared_ptr<Sensor>(new sensor_types::Magnetometer(sensor_config, this))});
        } else if(type == "motion_capture") {
            sensors.insert({name, std::shared_ptr<Sensor>(new sensor_types::MotionCapture(sensor_config, this))});
        } else if(type == "pitot") {
            sensors.insert({name, std::shared_ptr<Sensor>(new sensor_types::Pitot(sensor_config, this))});
        } else if(type == "rc_channels") {
            sensors.insert({name, std::shared_ptr<Sensor>(new sensor_types::RcChannels(sensor_config, this))});
        } else if(type == "thermometer") {
            sensors.insert({name, std::shared_ptr<Sensor>(new sensor_types::Thermometer(sensor_config, this))});
        }
        else {
            throw std::runtime_error("Sensor type '"+type+"' has not been implemented.");
        }
    }
}

//////////////////////////////////////////////////

void Craft::update(Cmd const& cmd) {
    // Update kill status
    if(cmd.kill.given()) {killed = cmd.kill;}
    // For each sensor
    for(auto & pair : sensors) {
        // If a command for that sensor was given, update the sensor with said command
        if(cmd.sensor_cmds.count(pair.first)) {pair.second->update(cmd.sensor_cmds.at(pair.first));}
        // Otherwise update with a passive command
        else {pair.second->update(Sensor::Cmd());}
    }
    // Update the body state
    body.update(cmd.body_cmd);
    // Prepare to record total power usage
    power = 0;
    // For each actuator
    for(auto & pair : actuators) {
        // Extract the actuator's name and pointer
        Str const& name = pair.first;
        std::shared_ptr<Actuator> actuator = pair.second;
        // If there is energy left and not killed
        if((energy > EPS) && !killed) {
            // Drain energy by power consumption for the timestep
            energy -= actuator->power * world->clock.dt;
            power += actuator->power;
            // If a command for that actuator was given, update the actuator with said command
            if(cmd.actuator_cmds.count(name)) {actuator->update(cmd.actuator_cmds.at(name));}
            // Otherwise update with a passive command
            else {actuator->update(Actuator::Cmd());}
        // Otherwise if the remaining energy is negative
        } else {
            // Actuator is killed and its physics are updated with a passive command
            // (Even dead surfaces will produce force! They just can't be controlled.)
            actuator->kill();
            actuator->update(Actuator::Cmd());
        }
    }
    // Drain energy by craft computer and/or other non-actuator power consumption for the timestep
    if(energy > EPS) {
        energy -= power0 * world->clock.dt;
        power += power0;
    } else {
        energy = 0;
    }
}

//////////////////////////////////////////////////

} // namespace physics
} // namespace stokesim
