/** @file
Typical main executable for using a stokesim::Simulator.
*/
#include "stokesim/simulator.h"

//////////////////////////////////////////////////

/// Constructs and runs a simulation given a config file as a command-line argument
int main(int argc, char** argv) {
    std::cout << "========================" << std::endl;
    std::cout << "       ~STOKESIM~       " << std::endl;
    std::cout << "========================" << std::endl;

    // Parse input arguments
    if(argc < 2) {
        std::cerr << "Please pass in the path to a proper YAML config file for a stokesim::Simulator." << std::endl;
        return 1;
    }
    std::string config_file = argv[1];
    std::string option = "none";
    if(argc > 2) {option = argv[2];}

    // Load config YAML
    std::cout << "Loading configuration from '" << config_file << "'..." << std::endl;
    YAML::Node config = YAML::LoadFile(config_file);

    // Create and run simulation
    stokesim::Simulator sim(config);
    int exit_status;
    if(option == "profile") {
        exit_status = sim.profile();
    } else if(option == "none") {
        exit_status = sim.simulate();
    } else {
        std::cerr << "FATAL: main execution option '" + option + "' not recognized." << std::endl;
        return 2;
    }

    // Verify success
    if(exit_status == 0) {
        std::cout << "Simulator exited with SUCCESS status." << std::endl;
    } else {
        std::cerr << "Simulator exited with FAILURE status (" << exit_status << ")." << std::endl;
    }
    return exit_status;
}

//////////////////////////////////////////////////
