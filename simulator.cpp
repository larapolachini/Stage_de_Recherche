#include <iostream>
#include <cstdlib> // For exit()

#include <yaml-cpp/yaml.h>
#include <unordered_map>

#include "simulator.h"
#include "spogobot.h"
#undef main         // We defined main() as robot_main() in pogobot.h

#include <iostream>
#include <string>
#include <unordered_map>
#include <yaml-cpp/yaml.h>


// Load configuration from a YAML file
void Configuration::load(const std::string& file_name) {
    try {
        YAML::Node yaml_config = YAML::LoadFile(file_name);

        for (const auto& item : yaml_config) {
            std::string key = item.first.as<std::string>();
            std::string value = item.second.as<std::string>();
            config_map[key] = value;
        }
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("Error reading the YAML file: " + std::string(e.what()));
    }
}

// Get a configuration value by key
std::string Configuration::get(const std::string& key, const std::string& default_value) const {
    auto it = config_map.find(key);
    if (it != config_map.end()) {
        return it->second;
    }
    return default_value;
}

// Check if a key exists in the configuration
bool Configuration::contains(const std::string& key) const {
    return config_map.find(key) != config_map.end();
}

// Display all configuration parameters
void Configuration::print() const {
    std::cout << "Configuration Parameters:" << std::endl;
    for (const auto& [key, value] : config_map) {
        std::cout << key << ": " << value << std::endl;
    }
}

bool parse_arguments(int argc, char* argv[], std::string& config_file, bool& verbose) {
    verbose = false;
    config_file.clear();

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "-c") {
            if (i + 1 < argc) {
                config_file = argv[++i];
            } else {
                std::cerr << "Error: -c requires a configuration file argument." << std::endl;
                return false;
            }
        } else if (arg == "-v") {
            verbose = true;
        } else {
            std::cerr << "Unknown argument: " << arg << std::endl;
            return false;
        }
    }

    return true;
}



void create_robots(Configuration& config) {
    uint32_t const nb_robots = std::stoi(config.get("nBots", "100"));
    if (!nb_robots)
        throw std::runtime_error("Number of robots is 0 (nBot=0 in configuration).");

    for (size_t i = 0; i < nb_robots; ++i) {
        robots.emplace_back();
    }
    current_robot = &robots.front();
}


void main_loop(Configuration& config) {
    uint32_t const simulation_time = std::stoi(config.get("simulationTime", "100"));

    // Launch main() on all robots
    for (auto& robot : robots) {
        current_robot = &robot;
        robot.data = malloc(UserdataSize);
        mydata = robot.data;
        robot_main();
    }

    // Setup all robots
    for (auto& robot : robots) {
        current_robot = &robot;
        mydata = robot.data;
        current_robot->user_setup();
    }

    // Main loop for all robots
    for (uint32_t i = 0; i < simulation_time; ++i) {
        for (auto& robot : robots) {
            current_robot = &robot;
            mydata = robot.data;
            current_robot->user_loop();
        }
    }

    // Free robot data
    for (auto& robot : robots) {
        free(robot.data);
    }
}

//extern "C" {
//    int __wrap_main(int argc, char** argv);
//    int __real_main();
//}
//
//
//int __wrap_main(int argc, char** argv) {
//    std::cout << "Starting the program from C++...\n";
//
//    // Perform pre-main initialization or setup
//    std::cout << "Performing pre-main initialization.\n";
//
//    // Call the C `main` function
//    int exit_code = __real_main();
//
//    std::cout << "Exiting the program from C++...\n";
//
//    // Ensure proper program termination
//    exit(exit_code);
//}


int main(int argc, char** argv) {
    std::string config_file;
    bool verbose = false;

    // Parse command-line arguments
    if (!parse_arguments(argc, argv, config_file, verbose)) {
        std::cerr << "Usage: " << argv[0] << " -c CONFIG_FILE [-v]" << std::endl;
        return 1;
    }

    // Enable verbose mode if requested
    if (verbose) {
        std::cout << "Verbose mode enabled." << std::endl;
    }

    Configuration config;
    try {
        // Load configuration
        config.load(config_file);

        if (verbose) {
            std::cout << "Loaded configuration from: " << config_file << std::endl;
        }

        // Display configuration
        if (verbose)
            config.print();

//        // Example: Access specific parameters
//        std::string arena_file = config.get("arenaFileName", "default_arena.csv");
//        int n_bots = std::stoi(config.get("nBots", "100"));
//
//        std::cout << "\nArena File: " << arena_file << std::endl;
//        std::cout << "Number of Bots: " << n_bots << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "Starting the program from C++...\n";

    // Perform pre-main initialization or setup
    std::cout << "Performing pre-main initialization.\n";

    // Create the robots
    create_robots(config);

    // Launch simulation
    main_loop(config);

//    // Call the C `main` function
//    int exit_code = robot_main();

    std::cout << "Exiting the program from C++...\n";

    // Ensure proper program termination
    //exit(exit_code);
    exit(0);
}

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
