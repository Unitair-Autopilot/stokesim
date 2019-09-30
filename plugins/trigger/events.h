/** @file
These are the possible events that a Trigger can check for.
They derive from an abstract base class called Event.
*/
#include "stokesim/io/plugin.h"

namespace trigger {

//////////////////////////////////////////////////

/// Base class so that the Trigger plugin can just run through a list of Event-derived objects
class Event {
private:
    /// @name Provided
    //@{
    std::string const type;
    //@}

public:
    /// @name Core
    //@{
    Event(YAML::Node const& config) : type(stokesim::yget("type", config).as<std::string>()) {}
    /// Checks something in the world, fills out the given Output accordingly, and returns whether it is done (true) or needs to check again (false)
    virtual bool evaluate(stokesim::io::Plugin::Output & output, stokesim::physics::World const& world) =0;
    //@}
};

//////////////////////////////////////////////////

/// Kills a specified craft after a specified amount of simulation time
class KillCraft : public Event {
private:
    /// @name Provided
    //@{
    std::string const craft_name;
    stokesim::Escal const time;
    //@}

public:
    /// @name Core
    //@{
    KillCraft(YAML::Node const& config) :
        Event(config),
        craft_name(stokesim::yget("craft_name", config).as<std::string>()),
        time(stokesim::yget("time", config).as<stokesim::Escal>()) {
    }

    bool evaluate(stokesim::io::Plugin::Output & output, stokesim::physics::World const& world) override {
        if(world.clock.t >= time) {
            output.world_cmd.craft_cmds[craft_name].kill = true;
            stokesim::print("KillCraft was triggered for '"+craft_name+"'!");
            return true;
        }
        return false;
    }
    //@}
};

//////////////////////////////////////////////////

/// Kills a specified craft's sensor after a specified amount of simulation time
class KillSensor : public Event {
private:
    /// @name Provided
    //@{
    std::string const craft_name;
    std::string const sensor_name;
    stokesim::Escal const time;
    //@}

public:
    /// @name Core
    //@{
    KillSensor(YAML::Node const& config) :
        Event(config),
        craft_name(stokesim::yget("craft_name", config).as<std::string>()),
        sensor_name(stokesim::yget("sensor_name", config).as<std::string>()),
        time(stokesim::yget("time", config).as<stokesim::Escal>()) {
    }

    bool evaluate(stokesim::io::Plugin::Output & output, stokesim::physics::World const& world) override {
        if(world.clock.t >= time) {
            output.world_cmd.craft_cmds[craft_name].sensor_cmds[sensor_name].kill = true;
            stokesim::print("KillSensor was triggered for '"+sensor_name+"' of '"+craft_name+"'!");
            return true;
        }
        return false;
    }
    //@}
};

//////////////////////////////////////////////////

/// Generates a global gust of wind at a specified simulation time for a specified duration
class Gust : public Event {
private:
    /// @name Provided
    //@{
    stokesim::Escal const speed;
    stokesim::Evec3 const direction;
    stokesim::Escal const time;
    stokesim::Escal const duration;
    //@}

    /// @name Derived
    //@{
    stokesim::Escal const end_time;
    bool active;
    //@}

public:
    /// @name Core
    //@{
    Gust(YAML::Node const& config) :
        Event(config),
        speed(stokesim::yget("speed", config).as<stokesim::Escal>()),
        direction(stokesim::evec_from_yaml(stokesim::yget("direction", config), 3).normalized()),
        time(stokesim::yget("time", config).as<stokesim::Escal>()),
        duration(stokesim::yget("duration", config).as<stokesim::Escal>()),
        end_time(time + duration),
        active(false) {
    }

    bool evaluate(stokesim::io::Plugin::Output & output, stokesim::physics::World const& world) override {
        if(world.clock.t > end_time) {
            output.world_cmd.env_cmd.atmos_cmd.gust_mag = 0;
            if(active) {
                stokesim::print("Gust finished.");
                active = false;
            }
            return true;
        }
        if(world.clock.t >= time) {
            output.world_cmd.env_cmd.atmos_cmd.gust_mag = speed;
            output.world_cmd.env_cmd.atmos_cmd.gust_dir = direction;
            if(!active) {
                stokesim::print("Gust triggered!");
                active = true;
            }
        }
        return false;
    }
    //@}
};

//////////////////////////////////////////////////

/// Disables GPS sensing for any craft within a specified rectangular geo-fence
class DenyGPS : public Event {
private:
    /// @name Provided
    //@{
    stokesim::LLA const lla_min; /// "Bottom-left" corner of GPS-denial geo-fence
    stokesim::LLA const lla_max; /// "Top-right" corner of GPS-denial geo-fence
    //@}

    /// @name Derived
    //@{
    stokesim::Dict<stokesim::Str> gps_users; /// Keys are GPS-using craft names and values are their GPS sensor names
    stokesim::Dict<bool> victims; /// Keys are GPS-using craft names and values are whether they are being denied
    //@}

public:
    /// @name Core
    //@{
    DenyGPS(YAML::Node const& config, stokesim::physics::World const& world) :
        Event(config),
        lla_min(stokesim::lla_from_yaml(stokesim::yget("lla_min", config))),
        lla_max(stokesim::lla_from_yaml(stokesim::yget("lla_max", config))) {
        // Find all crafts that have a GPS sensor
        for(auto const& craft_name : world.craft_names) {
            auto const& craft = world.get_craft(craft_name);
            for(auto const& sensor_name : craft.sensor_names) {
                auto const& sensor = craft.get_sensor(sensor_name);
                if(sensor.type == "gps") {
                    gps_users[craft_name] = sensor_name;
                    victims[craft_name] = false;
                }
            }
        }
    }

    bool evaluate(stokesim::io::Plugin::Output & output, stokesim::physics::World const& world) override {
        for(auto const& gps_user : gps_users) {
            stokesim::Str const& craft_name = gps_user.first;
            stokesim::Str const& gps_name = gps_user.second;
            stokesim::LLA const& lla = world.get_craft(craft_name).body.lla;
            if(((lla_min.lat <= lla.lat) && (lla.lat <= lla_max.lat)) &&
               ((lla_min.lon <= lla.lon) && (lla.lon <= lla_max.lon)) &&
               ((lla_min.alt <= lla.alt) && (lla.alt <= lla_max.alt))) {
                if(!victims[craft_name]) {
                    stokesim::print("DenyGPS was triggered for '"+gps_name+"' of '"+craft_name+"'!");
                    victims[craft_name] = true;
                }
                output.world_cmd.craft_cmds[craft_name].sensor_cmds[gps_name].kill = true;
            } else if(victims[craft_name]) {
                stokesim::print("DenyGPS was untriggered for '"+gps_name+"' of '"+craft_name+"'.");
                victims[craft_name] = false;
                output.world_cmd.craft_cmds[craft_name].sensor_cmds[gps_name].kill = false;
            }
        }
        return false;
    }
    //@}
};

//////////////////////////////////////////////////

} // namespace trigger
