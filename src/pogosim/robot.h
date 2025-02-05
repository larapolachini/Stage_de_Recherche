#ifndef ROBOT_H
#define ROBOT_H

#include <vector>
#include <set>
#include <queue>
#include <chrono>
#include <SDL2/SDL.h>
#include <box2d/box2d.h>

#include "utils.h"
#include "render.h"
#include "colormaps.h"
#include "spogobot.h"

std::string log_current_robot();


class Robot {
public:
    Robot(uint16_t _id, size_t _userdatasize, float x, float y, float _radius, b2WorldId worldId, float _msg_success_rate=0.5);
    //virtual ~Robot();

    // Base info
    uint16_t id;
    void* data = nullptr;
    void (*user_init)(void) = nullptr;
    void (*user_step)(void) = nullptr;
    void (*callback_create_data_schema)(void) = nullptr;
    void (*callback_export_data)(void) = nullptr;
    void launch_user_step();

    std::chrono::time_point<std::chrono::system_clock> current_time;
    uint64_t current_time_microseconds = 0LL;

    // C-code accessible values
    uint32_t pogobot_ticks = 0;
    uint8_t main_loop_hz = 60;
    uint8_t max_nb_processed_msg_per_tick = 3;
    void (*msg_rx_fn)(message_t*) = nullptr;
    bool (*msg_tx_fn)(void) = nullptr;
    int8_t error_codes_led_idx = 3;
    time_reference_t _global_timer;
    time_reference_t timer_main_loop;
    uint32_t _current_time_milliseconds = 0;
    uint32_t _error_code_initial_time = 0;
    uint8_t percent_msgs_sent_per_ticks = 20;
    uint32_t nb_msgs_sent = 0;
    uint32_t nb_msgs_recv = 0;

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
    void set_motor(motor_id motor, int speed);
    b2Vec2 get_position() const;
    float get_angle() const;

    // LEDs
    std::vector<color_t> leds = std::vector<color_t>(5, {0, 0, 0});

    // Neighbors
    std::vector<Robot*> neighbors;
    std::queue<message_t> messages;
    void send_to_neighbors(short_message_t *const message);
    void send_to_neighbors(message_t *const message);

    // Messages
    float msg_success_rate = 0.5;

};

extern Robot* current_robot;
extern int UserdataSize;
extern void* mydata;

//extern std::chrono::time_point<std::chrono::system_clock> sim_starting_time;
extern uint64_t sim_starting_time_microseconds;
uint64_t get_current_time_microseconds();
#endif


// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
