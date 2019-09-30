/** @file
This plugin manages a set of events that can be triggered, resulting in some specified
change to the simulator at run-time.
*/
#include "stokesim/io/plugin.h"
#include "events.h"

namespace trigger {

//////////////////////////////////////////////////

/// Iterates over the specified events list and calls their evaluate functions.
class Trigger : public stokesim::io::Plugin {
private:
    /// @name Provided
    //@{
    std::vector<std::shared_ptr<Event>> events; ///< Pointers to the Event-derived objects that will be checked for triggering
    //@}

public:
    /// Implements Plugin::configure
    bool configure(YAML::Node const& config, stokesim::physics::World const& world) override {
        // Iterate over the event configs
        for(auto const& event_config : stokesim::yget("events", config)) {
            // Extract the type of the event
            std::string const& type = stokesim::yget("type", event_config).as<std::string>();
            // Construct and append the correct Event-derived object to the events list depending on the type
            if(type == "kill_craft") {
                events.push_back(std::shared_ptr<Event>(new KillCraft(event_config)));
            } else if(type == "kill_sensor") {
                events.push_back(std::shared_ptr<Event>(new KillSensor(event_config)));
            } else if(type == "gust") {
                events.push_back(std::shared_ptr<Event>(new Gust(event_config)));
            } else if(type == "deny_gps") {
                events.push_back(std::shared_ptr<Event>(new DenyGPS(event_config, world)));
            } else {
                throw std::runtime_error("Event type '"+type+"' has not been implemented.");
            }
        }
        return true;
    }

    /// Implements Plugin::work
    Output work(stokesim::physics::World const& world) override {
        Output output;
        // Iterate over events
        for(auto itr=events.begin(); itr!=events.end();) {
            // Evaluate the event
            if((*itr)->evaluate(output, world)) {
                // If it says its done, remove it from future consideration
                events.erase(itr);
            } else {
                // If it isn't done checking, just move on to the next event
                ++itr;
            }
        }
        return output;
    }
};

//////////////////////////////////////////////////

} // namespace trigger

// Generate factory functions
EXPORT_AS_PLUGIN(trigger::Trigger)
