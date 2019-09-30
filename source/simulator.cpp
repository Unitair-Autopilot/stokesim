/** @file
Implementation of simulator.h
*/
#include "stokesim/simulator.h"

namespace stokesim {

//////////////////////////////////////////////////

Simulator::Simulator(YAML::Node const& config) :
    realtime(yget("realtime", config).as<bool>()),
    end_time(yget("end_time", config).as<Escal>() < 0 ? INF : yget("end_time", config).as<Escal>()), // negative implies infinite horizon
    world(yget("world", config), this) {
    // For each plugin specification
    for(auto const& pair : yget("plugins", config)) {
        Str const& name = pair.first.as<Str>();
        YAML::Node const& plugin_config = pair.second;
        // Add the plugin name to the names list
        plugin_names.push_back(name);
        // Find and open the binary file defining the plugin
        Str bin_file = yget("bin_file", plugin_config).as<Str>();
        print("Loading plugin '"+name+"'...");
        void* handle = dlopen(bin_file.c_str(), RTLD_NOW);
        if(handle == nullptr) {
            std::cerr << dlerror() << std::endl;
            throw std::runtime_error("Failed to open plugin library '"+bin_file+"'.");
        }
        // Make a pointer to the plugin library's create() function
        io::Plugin::Creator* create = (io::Plugin::Creator*) dlsym(handle, "create");
        if(create == nullptr) {
            std::cerr << dlerror() << std::endl;
            throw std::runtime_error("Failed to find 'create()' function.");
        }
        // Call create() to dynamically allocate a plugin implementation class and store its pointer
        plugins.insert({name, std::shared_ptr<io::Plugin>(create())});
        // Run its configure routine
        if(!plugins.at(name)->configure_wrapper(plugin_config, world)) {
            throw std::runtime_error("Plugin configure function returned failure.");
        }
    }
}

//////////////////////////////////////////////////

int Simulator::simulate() {
    print("Simulation running...");
    print("========================");
    bool terminate = false;
    Escal start_time = query_realtime();
    while(!terminate) {
        // Catch up to realtime (or just enter loop if not tracking realtime)
        while(((world.clock.t < (query_realtime()-start_time)) || !realtime) && !terminate) {
            // Prepare to collect plugin outputs
            io::Plugin::Output output;
            // Check on each plugin
            for(auto const& pair : plugins) {
                Str const& name = pair.first;
                std::shared_ptr<io::Plugin> plugin = pair.second;
                // If a call_period worth of sim time has passed (microsecond precision) then...
                if(plugin->call_period - (world.clock.t-plugin->last_call) <= 1e-6) {
                    // Let the plugin do work
                    output.merge(plugin->work(world));
                    // Record work call time
                    plugin->last_call = world.clock.t;
                    // Update termination signal
                    if(output.terminate && !terminate) {
                        print("Plugin '"+name+"' signaled for end of sim!");
                        terminate = true;
                    }
                }
            }
            // Let the sim do work if end_time not exceeded
            if(world.clock.t >= end_time) {
                print("End-time reached! ("+std::to_string(end_time)+")");
                terminate = true;
            } else if(!terminate) {
                world.update(output.world_cmd);
            }
        }
        // Don't burn whole CPU
        sleep_us(10);
    }
    print("Simulation finished!");
    print("========================");
    // Nominal exit status
    return 0;
}

//////////////////////////////////////////////////

int Simulator::profile() {
    // Reject infinite simulations
    if(end_time >= INF-1) {
        throw std::runtime_error("Cannot profile an endless simulation.");
    }
    // Record start time
    Escal start_time = query_realtime();
    // Run simulation
    int exit_status = simulate();
    // Compute real-time elapsed since start
    Escal elapsed = query_realtime() - start_time;
    // Print performance information
    print("Timing Results");
    if(realtime) {print("(warning: realtime throttle was active)");}
    print("----");
    print("    Number of plugins: "+std::to_string(plugins.size()));
    print("     Number of crafts: "+std::to_string(world.crafts.size()));
    print(" Integration timestep: "+std::to_string(world.clock.dt));
    print("  Simulation end-time: "+std::to_string(end_time));
    print("      Real-time taken: "+std::to_string(elapsed)+" s");
    print("    Performance ratio: "+std::to_string(end_time/elapsed));
    print("Average step duration: "+std::to_string(1e6*world.clock.dt*elapsed/end_time)+" us/step");
    print("========================");
    return exit_status;
}

//////////////////////////////////////////////////

Escal Simulator::query_realtime() const {
    return 1e-6*std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::system_clock::now().time_since_epoch()).count();
}

//////////////////////////////////////////////////

void Simulator::sleep_us(int dt) const {
    std::this_thread::sleep_for(std::chrono::microseconds(dt));
}

//////////////////////////////////////////////////

int Simulator::test() const {
    std::runtime_error("Unit-tests not yet implemented.");
    return -1;
}

//////////////////////////////////////////////////

} // namespace stokesim

stokesim::Simulator* create_simulator(char const config_file[]) {
    YAML::Node config = YAML::LoadFile(std::string(config_file));
    return new stokesim::Simulator(config);
}
void destroy_simulator(stokesim::Simulator* ptr) {
    delete ptr;
}
