#ifndef POGOBOT_H
#define POGOBOT_H

#include <stdint.h>
#include <stdbool.h>

// Create a macro to specify that we are using a simulator, and not real Pogobots
#define SIMULATOR

// Rename the main function from the robot code. The simulator uses a different main function.
#define main robot_main

#define motorL "left"
#define motorR "right"
#define motorFull 100
#define motorStop 0

#ifdef __cplusplus
extern "C" {
#endif

// TODO define all functions of the pogobot API

struct Robot;

typedef struct time_reference_t {
#ifdef __cplusplus
    void reset();
    uint32_t get_elapsed_microseconds();
    void enable();
    void disable();
    void add_elapsed_microseconds(uint64_t microseconds);
#endif

    bool enabled;
    uint32_t hardware_value_at_time_origin;
    //std::chrono::time_point<std::chrono::system_clock> start_time;
    uint64_t start_time;
    uint32_t elapsed_ms;
} time_reference_t;

void pogobot_init(void);
uint16_t pogobot_helper_getid(void);
void pogobot_stopwatch_reset(time_reference_t *stopwatch);
int32_t pogobot_stopwatch_get_elapsed_microseconds(time_reference_t *stopwatch);
void pogobot_led_setColor(const uint8_t r, const uint8_t g, const uint8_t b);
void pogobot_led_setColors(const uint8_t r, const uint8_t g, const uint8_t b, uint8_t id);
void pogobot_motor_set(const char* motor, int speed);
void msleep(int milliseconds);
void pogosim_printf(const char* format, ...);

#ifdef __cplusplus
}
#endif

#ifndef __cplusplus
// Define custom printf as the default in files including pogosim.h
#define printf pogosim_printf
#endif


#ifdef __cplusplus

#include <vector>
#include <set>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h> // For colored console output
#include <spdlog/fmt/ostr.h> // Enables << operator for logging
#include <chrono>
#include <SDL2/SDL.h>
#include <box2d/box2d.h>

#include "render.h"
#include "colormaps.h"

// XXX Move ??
// Declare a global logger (shared pointer)
extern std::shared_ptr<spdlog::logger> glogger;
void init_logger();
std::string log_current_robot();


class Robot {
public:
    Robot(uint16_t _id, size_t _userdatasize, float x, float y, float _radius, b2WorldId worldId);
    //virtual ~Robot();

    std::chrono::time_point<std::chrono::system_clock> current_time;
    uint64_t current_time_microseconds = 0LL;

    uint32_t pogobot_ticks = 0;
    uint8_t main_loop_hz = 60;
    uint8_t send_msg_hz = 60;
    uint8_t process_msg_hz = 60;
    void (*msg_rx_fn)(void) = nullptr;
    void (*msg_tx_fn)(void) = nullptr;
    int8_t error_codes_led_idx = 3;
    time_reference_t _global_timer;
    time_reference_t timer_main_loop;
    uint64_t _current_time_milliseconds = 0LL;

    uint16_t id;
    void* data = nullptr;
    void (*user_init)(void) = nullptr;
    void (*user_step)(void) = nullptr;
    void launch_user_step();

    // Time-related stuff
    std::set<time_reference_t*> stop_watches;
    void update_time();
    void register_stop_watch(time_reference_t* sw);
    void enable_stop_watches();
    void disable_stop_watches();

    // Physical information
    b2BodyId bodyId;
    b2ShapeId shapeId;
    float radius = 5;
    float left_motor_speed  = 0;
    float right_motor_speed = 0;
    void create_body(b2WorldId worldId, float x, float y);
    void render(SDL_Renderer* renderer, b2WorldId worldId) const;
    void set_motor(const char* motor, int speed);

    // LEDs
    std::vector<color_t> leds = std::vector<color_t>(5, {0, 0, 0});

};

extern Robot* current_robot;
extern int UserdataSize;
extern void* mydata;

//extern std::chrono::time_point<std::chrono::system_clock> sim_starting_time;
extern uint64_t sim_starting_time_microseconds;
uint64_t get_current_time_microseconds();
#endif


#endif

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
