#ifndef SIMULATOR_H
#define SIMULATOR_H

//#include "spogobot.h"
#include "pogosim.h"
#include "configuration.h"

extern "C" int robot_main(void);

//void create_robots(Configuration& config);
//void main_loop(Configuration& config);
void set_current_robot(Robot& robot);

class Simulation {
    Configuration& config;

    // SDL Globals
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    bool enable_gui = true;
    bool paused = false;
    bool running = true;
    float t = 0.0f;

    uint16_t window_width = 800;
    uint16_t window_height = 600;
    uint16_t robot_radius = 10;
    uint16_t sub_step_count = 4;
    double GUI_speed_up = 1.0;
    float comm_radius = 90.0;

    float const wall_offset = 30.0f;
    float const minX = wall_offset + robot_radius*2;
    float const maxX = window_width - wall_offset - robot_radius*2;
    float const minY = wall_offset + robot_radius*2;
    float const maxY = window_height - wall_offset - robot_radius*2;

    b2WorldId worldId;
    std::vector<Robot> robots;
    std::vector<Robot> membranes;
    std::vector<std::vector<b2Vec2>> arena_polygons;

    float last_frame_saved_t = -1.0;

    int16_t current_light_value = std::numeric_limits<int16_t>::max();
    float photo_start_at = -1.f;
    float photo_start_duration = 1.f;

public:
    Simulation(Configuration& _config);
    virtual ~Simulation();

    void create_robots();
    void create_membranes();
    void create_arena();
    void create_walls();
    void init_box2d();
    void init_config();
    void init_SDL();
    void main_loop();
    void handle_SDL_events();
    void render_all();
    void export_frames();
    void delete_old_data();
    void compute_neighbors();

    void speed_up();
    void speed_down();
    void pause();
    void photo_start();

    uint16_t get_current_light_value() const;
};

extern std::unique_ptr<Simulation> simulation;

#endif // SIMULATOR_H

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
