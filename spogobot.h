#ifndef POGOBOT_H
#define POGOBOT_H

#include <stdint.h>

// TODO : version of the pogobot lib for pogosim

// Create a macro to specify that we are using a simulator, and not real Pogobots
#define SIMULATOR

// Rename the main function from the robot code. The simulator uses a different main function.
#define main robot_main

// TODO define the entire pogobot API

#define motorL "left"
#define motorR "right"
#define motorFull 100
#define motorStop 0

#ifdef __cplusplus
extern "C" {
#endif

// TODO define all functions of the pogobot API

typedef struct time_reference_t {
    uint32_t hardware_value_at_time_origin;
} time_reference_t;

void pogobot_init(void);
uint16_t pogobot_helper_getid(void);
void pogobot_stopwatch_reset( time_reference_t *stopwatch );
int32_t pogobot_stopwatch_get_elapsed_microseconds( time_reference_t *stopwatch );
void pogobot_led_setColor(int r, int g, int b);
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

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h> // For colored console output
#include <spdlog/fmt/ostr.h> // Enables << operator for logging



// Declare a global logger (shared pointer)
extern std::shared_ptr<spdlog::logger> glogger;

void init_logger();

class Robot {
public:
    Robot(uint16_t _id, size_t _userdatasize);
    //~Robot();


    long long current_time_microseconds = 0LL;
    long pogo_ticks = 0;

    uint16_t id;
    void* data = nullptr;
    void (*user_init)(void);
    void (*user_step)(void);
};


extern Robot* current_robot;
extern std::vector<Robot> robots;
extern int UserdataSize;
extern void* mydata;
extern uint64_t pogo_ticks;

std::string log_current_robot();
#endif


#endif

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
