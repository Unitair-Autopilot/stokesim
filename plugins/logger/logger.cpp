/** @file
This example plugin writes the state, measurements, and actuator force norms
of a specific craft out to a given log file in the following row format:
t,pos0,pos1,pos2,oriw,orix,oriy,oriz,vel0,vel1,vel2,avel0,avel1,avel2,sensor_measurement...,actuator...
The first row is a header that labels each column's type.
The second row is a header that labels each column's name.
*/
#include <fstream>
#include "stokesim/io/plugin.h"

namespace logger {

//////////////////////////////////////////////////

/// Writes the state, measurements, and actuator force norms of a specific craft to a file.
class Logger : public stokesim::io::Plugin {
private:
    /// @name Provided
    //@{
    std::string craft_name; ///< Name of the craft of interest to log
    std::string log_file;   ///< Path of the file to write the log to
    //@}

    /// @name Derived
    //@{
    std::ofstream stream; ///< Output stream that will connect to the log_file
    //@}

public:
    /// Implements Plugin::~Plugin
    ~Logger() override {
        // Close the log file
        stream.close();
    }

    /// Implements Plugin::configure
    bool configure(YAML::Node const& config, stokesim::physics::World const& world) override {
        // Read plugin-specific data from the config YAML with proper error handling
        craft_name = stokesim::yget("craft_name", config).as<std::string>();
        log_file = stokesim::yget("log_file", config).as<std::string>();
        // Verify that the world actually has the specified craft
        if(!world.has_craft(craft_name)) {
            std::cerr << "World is missing craft with name '" + craft_name + "'." << std::endl;
            return false;
        }
        // Try to open the log file
        stream.open(log_file);
        if(!stream.is_open()) {
            std::cerr << "Could not open '" + log_file + "' for logging." << std::endl;
            return false;
        }
        // Prepare the state part of the header strings
        std::string type_header = "time,state,state,state,state,state,state,state,state,state,state,state,state,state";
        std::string name_header = "t,pos0,pos1,pos2,oriw,orix,oriy,oriz,vel0,vel1,vel2,avel0,avel1,avel2";
        // Get the specific craft of interest from the world
        auto const& craft = world.get_craft(craft_name);
        // Iterate over its list of valid sensor names
        for(auto const& sensor_name : craft.sensor_names) {
            // Iterate over each of those sensors' lists of valid measurement names
            for(auto const& measurement_name : craft.get_sensor(sensor_name).measurement_names) {
                // Add the corresponding sensor measurement to the header strings
                type_header += ",sensor";
                name_header += "," + sensor_name + "_" + measurement_name;
            }
        }
        // Iterate over the craft's list of valid actuator names
        for(auto const& actuator_name : craft.actuator_names) {
            // Append to the header strings
            type_header += ",actuator";
            name_header += "," + actuator_name;
        }
        // Write the header strings
        stream << type_header << std::endl;
        stream << name_header << std::endl;
        // Success
        return true;
    }

    /// Implements Plugin::work
    Output work(stokesim::physics::World const& world) override {
        // Get the craft of interest from the world
        auto const& craft = world.get_craft(craft_name);
        // Alias the craft's body for brevity
        auto const& body = craft.body;
        // Write the world's clock time and craft's body state information to the log file
        stream << world.clock.t << ","
               << body.pos(0)   << "," << body.pos(1)  << "," << body.pos(2)  << ","
               << body.ori.w()  << "," << body.ori.x() << "," << body.ori.y() << "," << body.ori.z() << ","
               << body.vel(0)   << "," << body.vel(1)  << "," << body.vel(2)  << ","
               << body.avel(0)  << "," << body.avel(1) << "," << body.avel(2);
        // Iterate over the craft's list of valid sensors
        for(auto const& sensor_name : craft.sensor_names) {
            // Get the actual sensor object
            auto const& sensor = craft.get_sensor(sensor_name);
            // Iterate over its list of valid measurement names
            for(auto const& measurement_name : sensor.measurement_names) {
                // Request each measurement from the sensor and log it
                stream << "," << sensor.get_measurement(measurement_name);
            }
        }
        // Iterate over the craft's list of valid actuator names
        for(auto const& actuator_name : craft.actuator_names) {
            // Compute and log each actuator's current force norm
            stream << "," << craft.get_actuator(actuator_name).force.norm();
        }
        // Move log to next line and return passive plugin output
        stream << std::endl;
        return Output();
    }
};

//////////////////////////////////////////////////

} // namespace logger

// Generate factory functions
EXPORT_AS_PLUGIN(logger::Logger)
