#include <iostream>
#include <cstdlib> // For exit()
#include <string>
#include <chrono>

#include <yaml-cpp/yaml.h>
#include <unordered_map>
#include <spdlog/spdlog.h>

#include "simulator.h"
#include "spogobot.h"
#undef main         // We defined main() as robot_main() in pogobot.h




void create_robots(Configuration& config) {
    uint32_t const nb_robots = std::stoi(config.get("nBots", "100"));
    glogger->info("Creating {} robots", nb_robots);
    if (!nb_robots)
        throw std::runtime_error("Number of robots is 0 (nBot=0 in configuration).");

    for (size_t i = 0; i < nb_robots; ++i) {
        robots.emplace_back(i, UserdataSize);
    }
    current_robot = &robots.front();

    glogger->info("Initializing all robots...");
    // Launch main() on all robots
    for (auto& robot : robots) {
        set_current_robot(robot);
        robot_main();
    }

    // Setup all robots
    for (auto& robot : robots) {
        set_current_robot(robot);
        current_robot->user_init();
    }
}

void set_current_robot(Robot& robot) {
    current_robot = &robot;
    mydata = robot.data;
    pogo_ticks = robot.pogo_ticks;

    // Update robot clock and handle time-keeping
    // TODO
}

void main_loop(Configuration& config) {
    uint32_t const simulation_time = std::stoi(config.get("simulationTime", "100"));
    glogger->info("Launching the main simulation loop.");

    //sim_starting_time = std::chrono::system_clock::now();
    sim_starting_time_microseconds = get_current_time_microseconds();

    // Main loop for all robots
    for (uint32_t i = 0; i < simulation_time; ++i) {
        for (auto& robot : robots) {
            set_current_robot(robot);
            robot.launch_user_step();
        }
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


int main(int argc, char** argv) {
    std::string config_file;
    bool verbose = false;

    // Parse command-line arguments
    if (!parse_arguments(argc, argv, config_file, verbose)) {
        std::cerr << "Usage: " << argv[0] << " -c CONFIG_FILE [-v]" << std::endl;
        return 1;
    }

    // Init logging
    init_logger();

    // Enable verbose mode if requested
    if (verbose) {
        glogger->info("Verbose mode enabled.");
        glogger->set_level(spdlog::level::debug);
    }

    Configuration config;
    try {
        // Load configuration
        config.load(config_file);

        if (verbose) {
            //glogger->info << "Loaded configuration from: " << config_file << std::endl;
            glogger->info("Loaded configuration from: {}", config_file);
        }

        // Display configuration
        if (verbose)
            glogger->debug(config.summary());

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

//    std::cout << "Starting the program from C++...\n";

    // Perform pre-main initialization or setup
//    std::cout << "Performing pre-main initialization.\n";

    // Create the robots
    create_robots(config);

    // Launch simulation
    main_loop(config);

//    // Call the C `main` function
//    int exit_code = robot_main();

//    std::cout << "Exiting the program from C++...\n";

    // Ensure proper program termination
    //exit(exit_code);
    exit(0);
}

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
