#include <iostream>
#include <string>
#include <chrono>

#include <yaml-cpp/yaml.h>
#include <unordered_map>
#include <spdlog/spdlog.h>

#include <cmath>
#include <vector>
#include <SDL2/SDL.h>
#include <box2d/box2d.h>

#include "utils.h"
#include "simulator.h"
#include "render.h"
#include "spogobot.h"
#undef main         // We defined main() as robot_main() in pogobot.h


// TODO Move into Robot class?
void set_current_robot(Robot& robot) {
    // Store values of previous robot
    if (current_robot != nullptr) {
        current_robot->pogobot_ticks        = pogobot_ticks;
        current_robot->main_loop_hz         = main_loop_hz;
        current_robot->send_msg_hz          = send_msg_hz;
        current_robot->process_msg_hz       = process_msg_hz;
        current_robot->msg_rx_fn            = msg_rx_fn;
        current_robot->msg_tx_fn            = msg_tx_fn;
        current_robot->error_codes_led_idx  = error_codes_led_idx;
        current_robot->_global_timer        = _global_timer;
        current_robot->timer_main_loop      = timer_main_loop;
        current_robot->_current_time_milliseconds = _current_time_milliseconds;
    }

    current_robot = &robot;
    mydata = robot.data;

    // Update robot values
    pogobot_ticks       = robot.pogobot_ticks;
    main_loop_hz        = robot.main_loop_hz;
    send_msg_hz         = robot.send_msg_hz;
    process_msg_hz      = robot.process_msg_hz;
    msg_rx_fn           = robot.msg_rx_fn;
    msg_tx_fn           = robot.msg_tx_fn;
    error_codes_led_idx = robot.error_codes_led_idx;
    _global_timer       = robot._global_timer;
    timer_main_loop     = robot.timer_main_loop;
    _current_time_milliseconds = robot._current_time_milliseconds;

    // Update robot clock and handle time-keeping
    // TODO
}


/************* SIMULATION *************/ // {{{1

Simulation::Simulation(Configuration& _config)
        : config(_config) {
    init_config();
    init_box2d();
    if (enable_gui)
        init_SDL();
    //create_walls();
    create_arena();
    create_robots();
    create_membranes();
}

Simulation::~Simulation() {
    b2DestroyWorld(worldId);
    if (enable_gui) {
        if (renderer)
            SDL_DestroyRenderer(renderer);
        if (window)
            SDL_DestroyWindow(window);
        SDL_Quit();
    }
}

// TODO
void Simulation::create_membranes() {
}


void Simulation::create_arena() {
    std::string const csv_file = config.get("arena_file", "test.csv");

    float const friction = 0.01f;
    float const restitution = 10.8f; // Bounciness
    float const WALL_THICKNESS = 20.0f / VISUALIZATION_SCALE; // Thickness of the wall in SDL units

    // Read multiple polygons from the CSV file
    arena_polygons = read_poly_from_csv(csv_file, window_width, window_height);
    if (arena_polygons.empty()) {
        glogger->error("Error: No polygons found in the arena file");
        throw std::runtime_error("No polygons found in the arena file or unable to open arena file");
    }

    // Process each polygon
    for (const auto& polygon : arena_polygons) {
        if (polygon.size() < 2) {
            std::cerr << "Error: A polygon must have at least two points to create walls." << std::endl;
            continue;
        }

        std::vector<b2Vec2> outer_polygon = offset_polygon(polygon, -1.0f * WALL_THICKNESS);

        // Define the static body for each wall segment
        b2BodyDef wallBodyDef = b2DefaultBodyDef();
        wallBodyDef.type = b2_staticBody;

        b2Vec2 p1;
        b2Vec2 p2;
        for (size_t i = 0; i < outer_polygon.size(); ++i) {
            if (i < outer_polygon.size() - 1) {
                p1 = outer_polygon[i];
                p2 = outer_polygon[i + 1];
            } else {
                p1 = outer_polygon[i];
                p2 = outer_polygon[0];
            }

            // Calculate the center of the rectangle
            b2Vec2 center = (p1 + p2) * 0.5f * (1.0f/VISUALIZATION_SCALE);

            // Calculate the angle of the rectangle
            float angle = atan2f(p2.y - p1.y, p2.x - p1.x);

            // Calculate the length of the rectangle
            float length = b2Distance(p1, p2) / VISUALIZATION_SCALE;

            // Create the wall body
            wallBodyDef.position = center;
            wallBodyDef.rotation = b2MakeRot(angle);
            b2BodyId wallBody = b2CreateBody(worldId, &wallBodyDef);

            // Create the rectangular shape
            b2Polygon wallShape = b2MakeBox(length / 2, WALL_THICKNESS / 2);
            b2ShapeDef wallShapeDef = b2DefaultShapeDef();
            wallShapeDef.friction = friction;
            wallShapeDef.restitution = restitution;

            b2CreatePolygonShape(wallBody, &wallShapeDef, &wallShape);
        }
    }

    glogger->info("Arena walls created from CSV file: {}", csv_file);
}


void Simulation::create_walls() {
    float const WALL_THICKNESS = 30.0f / VISUALIZATION_SCALE; // Thickness of the wall in Box2D units (30 pixels)
    float const offset = 30.0f / VISUALIZATION_SCALE;        // Offset from the window edge in Box2D units
    float const width = (window_width - 2 * 30) / VISUALIZATION_SCALE; // Width adjusted for 30-pixel offset
    float const height = (window_height - 2 * 30) / VISUALIZATION_SCALE; // Height adjusted for 30-pixel offset
    float const friction = 0.03f;
    float const restition = 10.8f; // Bounciness

    // Define the static body for each wall
    b2BodyDef wallBodyDef = b2DefaultBodyDef();
    wallBodyDef.type = b2_staticBody;

    // Bottom wall
    wallBodyDef.position = {offset + width / 2, offset - WALL_THICKNESS / 2};
    b2BodyId bottomWall = b2CreateBody(worldId, &wallBodyDef);

    b2Polygon bottomShape = b2MakeBox(width / 2, WALL_THICKNESS / 2);
    b2ShapeDef bottomShapeDef = b2DefaultShapeDef();
    bottomShapeDef.friction = friction;
    bottomShapeDef.restitution = restition;
    b2CreatePolygonShape(bottomWall, &bottomShapeDef, &bottomShape);

    // Top wall
    wallBodyDef.position = {offset + width / 2, offset + height + WALL_THICKNESS / 2};
    b2BodyId topWall = b2CreateBody(worldId, &wallBodyDef);
    b2Polygon topShape = b2MakeBox(width / 2, WALL_THICKNESS / 2);
    b2ShapeDef topShapeDef = b2DefaultShapeDef();
    topShapeDef.friction = friction;
    topShapeDef.restitution = restition;
    b2CreatePolygonShape(topWall, &topShapeDef, &topShape);

    // Left wall
    wallBodyDef.position = {offset - WALL_THICKNESS / 2, offset + height / 2};
    b2BodyId leftWall = b2CreateBody(worldId, &wallBodyDef);
    b2Polygon leftShape = b2MakeBox(WALL_THICKNESS / 2, height / 2);
    b2ShapeDef leftShapeDef = b2DefaultShapeDef();
    leftShapeDef.friction = friction;
    leftShapeDef.restitution = restition;
    b2CreatePolygonShape(leftWall, &leftShapeDef, &leftShape);

    // Right wall
    wallBodyDef.position = {offset + width + WALL_THICKNESS / 2, offset + height / 2};
    b2BodyId rightWall = b2CreateBody(worldId, &wallBodyDef);
    b2Polygon rightShape = b2MakeBox(WALL_THICKNESS / 2, height / 2);
    b2ShapeDef rightShapeDef = b2DefaultShapeDef();
    rightShapeDef.friction = friction;
    rightShapeDef.restitution = restition;
    b2CreatePolygonShape(rightWall, &rightShapeDef, &rightShape);
}


void Simulation::init_box2d() {
    // Initialize Box2D world
    b2Vec2 gravity = {0.0f, 0.0f}; // No gravity for robots
    b2WorldDef worldDef = b2DefaultWorldDef();
    worldDef.gravity = gravity;
    worldId = b2CreateWorld(&worldDef);
}


void Simulation::init_config() {
    window_width = std::stoi(config.get("window_width", "800"));
    window_height = std::stoi(config.get("window_height", "800"));
    robot_radius = std::stoi(config.get("robot_radius", "10"));
    enable_gui = string_to_bool(config.get("GUI", "true"));
    GUI_speed_up = std::stof(config.get("GUI_speed_up", "1.0"));
}


void Simulation::init_SDL() {
    // Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        SDL_Log("Failed to initialize SDL: %s", SDL_GetError());
        throw std::runtime_error("Error while initializing SDL");
    }

    window = SDL_CreateWindow("Swarm Robotics Simulator with Walls",
                                          SDL_WINDOWPOS_CENTERED,
                                          SDL_WINDOWPOS_CENTERED,
                                          window_width, window_height,
                                          SDL_WINDOW_SHOWN);
    if (!window) {
        SDL_Log("Failed to create window: %s", SDL_GetError());
        SDL_Quit();
        throw std::runtime_error("Error while initializing SDL");
    }

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        SDL_Log("Failed to create renderer: %s", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        throw std::runtime_error("Error while initializing SDL");
    }
}


void Simulation::create_robots() {
    uint32_t const nb_robots = std::stoi(config.get("nBots", "100"));
    glogger->info("Creating {} robots", nb_robots);
    if (!nb_robots)
        throw std::runtime_error("Number of robots is 0 (nBot=0 in configuration).");

    std::srand(std::time(nullptr));
    for (size_t i = 0; i < nb_robots; ++i) {
        auto const point = generate_random_point_within_polygon_safe(arena_polygons, 10.0 * robot_radius); // XXX quick & dirty :-/
        robots.emplace_back(i, UserdataSize, point.x, point.y, robot_radius, worldId);
        //float x = minX + std::rand() % static_cast<int>(maxX - minX);
        //float y = minY + std::rand() % static_cast<int>(maxY - minY);
        //robots.emplace_back(i, UserdataSize, x, y, robot_radius, worldId);
        glogger->debug("Creating robot at ({}, {})", point.x, point.y);
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


void Simulation::speed_up() {
    GUI_speed_up *= 1.1;
    glogger->info("Setting GUI speed up to {}", GUI_speed_up);
}

void Simulation::speed_down() {
    GUI_speed_up *= 0.9;
    glogger->info("Setting GUI speed up to {}", GUI_speed_up);
}

void Simulation::pause() {
    paused = !paused;
}

void Simulation::handle_SDL_events() {
    if (!enable_gui)
        return;

    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_QUIT) {
            running = false;

        } else if (event.type == SDL_KEYDOWN) {
            switch (event.key.keysym.sym) {
                case SDLK_F3:
                    speed_down();
                    break;
                case SDLK_F4:
                    speed_up();
                    break;
                case SDLK_ESCAPE:
                    running = false;
                    break;
                case SDLK_SPACE:
                    pause();
                    break;
                case SDLK_UP:
                    // TODO
                    std::cout << "UP arrow key pressed!" << std::endl;
                    break;
                case SDLK_DOWN:
                    // TODO
                    std::cout << "DOWN arrow key pressed!" << std::endl;
                    break;
                case SDLK_LEFT:
                    // TODO
                    std::cout << "LEFT arrow key pressed!" << std::endl;
                    break;
                case SDLK_RIGHT:
                    // TODO
                    std::cout << "RIGHT arrow key pressed!" << std::endl;
                    break;
            }
        }
    }
}



void Simulation::render_all() {
    if (!enable_gui)
        return;
    SDL_SetRenderDrawColor(renderer, 200, 200, 200, 255); // Grey background
    SDL_RenderClear(renderer);

    //renderWalls(renderer); // Render the walls
    for(auto const& poly : arena_polygons) {
        draw_polygon(renderer, poly);
    }
    //membrane.render(renderer, worldId);

    for (auto const& robot : robots) {
        robot.render(renderer, worldId);
    }
    //SDL_RenderPresent(renderer);
}

void Simulation::export_frames() {
    if (!enable_gui)
        return;

    // If wanted, export to PNG
    float const save_video_period = std::stof(config.get("save_video_period", "-1.0"));
    std::string const frames_name = config.get("frames_name", "frames/f{:06.4f}.png");
    if (save_video_period > 0.0 && frames_name.size()) {
        //float const time_step_duration = std::stof(config.get("timeStep", "0.01667"));
        if (t >= last_frame_saved_t + save_video_period) {
            last_frame_saved_t = t;
            std::string formatted_filename = std::vformat(frames_name, std::make_format_args(t));
            save_window_to_png(renderer, window, formatted_filename);
        }
    }
}


void Simulation::main_loop() {
    // Delete old data, if needed
    delete_old_data();

    float const simulation_time = std::stof(config.get("simulationTime", "100.0"));
    glogger->info("Launching the main simulation loop.");

    float const time_step_duration = std::stof(config.get("timeStep", "0.01667"));
    //float const GUI_time_step_duration = std::stof(config.get("GUItimeStep", "0.01667"));

    //sim_starting_time = std::chrono::system_clock::now();
    sim_starting_time_microseconds = get_current_time_microseconds();

    // Main loop for all robots
    running = true;
    t = 0.0f;
    last_frame_saved_t = 0.0f - time_step_duration;
    while (running && t < simulation_time) {
        handle_SDL_events();

        for (auto& robot : robots) {
            set_current_robot(robot);
            robot.launch_user_step();
        }

        // Step the Box2D world
        b2World_Step(worldId, time_step_duration, sub_step_count);

        // Render
        render_all();
        export_frames();
        SDL_RenderPresent(renderer);

        // Delay and update time
        if (enable_gui) {
            SDL_Delay(time_step_duration / GUI_speed_up);
        }
        t += time_step_duration;
    }
}

void Simulation::delete_old_data() {
    bool const delete_old_files = string_to_bool(config.get("delete_old_files", "false"));
    if (delete_old_files) {
        std::string const frames_name = config.get("frames_name", "frames/f{:06.4f}.png");
        std::filesystem::path filePath(filename);
        std::filesystem::path directory = filePath.parent_path();
        delete_files_with_extension(directory, ".png", false);
    }
}


bool parse_arguments(int argc, char* argv[], std::string& config_file, bool& verbose, bool& gui) {
    verbose = false;
    gui = true;
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
        } else if (arg == "-g") {
            gui = false;
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
    bool gui = true;

    // Parse command-line arguments
    if (!parse_arguments(argc, argv, config_file, verbose, gui)) {
        std::cerr << "Usage: " << argv[0] << " -c CONFIG_FILE [-v] [-g]" << std::endl;
        return 1;
    }

    // Init logging
    init_logger();

    // Enable verbose mode if requested
    if (verbose) {
        glogger->info("Verbose mode enabled.");
        glogger->set_level(spdlog::level::debug);
    }

    if (gui) {
        glogger->info("GUI enabled.");
    }

    Configuration config;
    try {
        // Load configuration
        config.load(config_file);

        if (verbose) {
            glogger->info("Loaded configuration from: {}", config_file);
        }

        // Display configuration
        if (verbose)
            glogger->debug(config.summary());

        config.set("GUI", gui ? "true" : "false");

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    // Create the simulation object
    Simulation simulation = Simulation(config);

    // Launch simulation
    simulation.main_loop();
    return 0;
}

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
