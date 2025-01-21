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

    uint16_t const window_width = 800;
    uint16_t const window_height = 600;
    uint16_t const robot_radius = 10;
    uint16_t const sub_step_count = 4;

    float const wall_offset = 30.0f;
    float const minX = wall_offset + robot_radius*2;
    float const maxX = window_width - wall_offset - robot_radius*2;
    float const minY = wall_offset + robot_radius*2;
    float const maxY = window_height - wall_offset - robot_radius*2;

    b2WorldId worldId;
    std::vector<Robot> robots;
    std::vector<Robot> membranes;

public:
    Simulation(Configuration& _config);
    virtual ~Simulation();

    void create_robots();
    void create_membranes();
    void create_arena();
    void create_walls();
    void init_box2d();
    void init_SDL();
    void main_loop();
};


#endif // SIMULATOR_H

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
